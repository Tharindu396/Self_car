from collections import deque
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
except ImportError:
    torch = None


LANE_NAMES = ("left", "center", "right")


def _distance_in_lane(
    detections: Optional[Sequence[Dict]],
    lane: str,
) -> Optional[float]:
    if not detections:
        return None
    distances = [
        det.get("distance_m")
        for det in detections
        if det.get("lane") == lane and det.get("distance_m") is not None
    ]
    return min(distances) if distances else None


def ego_lane_blocked(
    detections: Optional[Sequence[Dict]], safety_gap_m: float
) -> bool:
    distance = _distance_in_lane(detections, "center")
    return distance is not None and distance < safety_gap_m


def free_on_left(detections: Optional[Sequence[Dict]], min_gap_m: float = 1.5) -> bool:
    distance = _distance_in_lane(detections, "left")
    return distance is None or distance > min_gap_m


def free_on_right(
    detections: Optional[Sequence[Dict]], min_gap_m: float = 1.5
) -> bool:
    distance = _distance_in_lane(detections, "right")
    return distance is None or distance > min_gap_m


def aligned_with_lane(lane_offset: float, lane_heading: float, tol: float = 0.08) -> bool:
    return abs(lane_offset) < tol and abs(lane_heading) < tol


def pid_on_lane(lane_offset: float, lane_heading: float, bias: str = "Follow") -> float:
    bias_map = {"ChangeLeft": -0.2, "ChangeRight": 0.2}
    bias_term = bias_map.get(bias, 0.0)
    angle = 0.55 * lane_heading + 0.45 * lane_offset + bias_term
    return float(np.clip(angle, -1.0, 1.0))


class _ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int, hidden_sizes: Tuple[int, ...]):
        super().__init__()
        layers: List[nn.Module] = []
        last = obs_dim
        for size in hidden_sizes:
            layers.append(nn.Linear(last, size))
            layers.append(nn.ReLU())
            last = size
        self.backbone = nn.Sequential(*layers)
        self.actor = nn.Linear(last, action_dim)
        self.critic = nn.Linear(last, 1)
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, obs: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self.backbone(obs)
        mean = self.actor(features)
        value = self.critic(features)
        return mean, self.log_std, value


class PPOLaneDecisionPolicy:
    """Small PPO agent that can be trained offline and deployed online."""

    def __init__(
        self,
        obs_dim: int,
        action_dim: int = 2,
        hidden_sizes: Tuple[int, ...] = (64, 64),
        weights_path: Optional[str] = None,
        device: Optional[str] = None,
    ):
        if torch is None:
            raise RuntimeError("PyTorch is required for the PPO policy.")
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.model = _ActorCritic(obs_dim, action_dim, hidden_sizes).to(self.device)
        if weights_path:
            try:
                ckpt = torch.load(weights_path, map_location=self.device)
                self.model.load_state_dict(ckpt)
            except OSError:
                pass
        self.model.eval()

    @staticmethod
    def is_available() -> bool:
        return torch is not None

    def act(self, obs: np.ndarray, deterministic: bool = False) -> np.ndarray:
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            mean, log_std, _ = self.model(obs_t)
            dist = torch.distributions.Normal(mean, log_std.exp())
            action = dist.mean if deterministic else dist.rsample()
        return action.squeeze(0).cpu().numpy()


def _normalize_distance(distance_m: Optional[float], max_range_m: float = 8.0) -> float:
    if distance_m is None:
        return 1.0
    return float(np.clip(distance_m / max_range_m, 0.0, 1.0))


def _build_observation(
    lane_offset: float,
    lane_heading: float,
    detections: Optional[Sequence[Dict]],
) -> np.ndarray:
    distances = {
        lane: _normalize_distance(_distance_in_lane(detections, lane))
        for lane in LANE_NAMES
    }
    return np.array(
        [
            float(np.clip(lane_offset, -1.0, 1.0)),
            float(np.clip(lane_heading, -1.0, 1.0)),
            distances["left"],
            distances["center"],
            distances["right"],
        ],
        dtype=np.float32,
    )


def _heuristic_action(
    lane_offset: float,
    lane_heading: float,
    detections: Optional[Sequence[Dict]],
    max_throttle: float,
) -> Tuple[float, float]:
    min_center = _distance_in_lane(detections, "center") or 10.0
    desired_angle = pid_on_lane(lane_offset, lane_heading)
    slow_down = 1.0 if min_center < 1.0 else min(1.0, min_center / 3.0)
    throttle = max_throttle * slow_down
    if min_center < 0.6:
        throttle = 0.0
    return desired_angle, float(np.clip(throttle, 0.0, max_throttle))


class LaneChangePlanner:
    """Planner that fuses lane detections, object detections, and PPO policy."""

    def __init__(
        self,
        max_throttle: float = 0.25,
        safety_gap_m: float = 1.2,
        ppo_weights: Optional[str] = None,
        history: int = 3,
        stop_distance_m: float = 0.5,
    ):
        self.state = "Follow"
        self.max_throttle = max_throttle
        self.safety_gap_m = safety_gap_m
        self.stop_distance_m = stop_distance_m
        self.history = deque(maxlen=history)
        obs_dim = 5 * history if history > 1 else 5
        self.policy = None
        if PPOLaneDecisionPolicy.is_available():
            self.policy = PPOLaneDecisionPolicy(obs_dim, weights_path=ppo_weights)

    def _stack_observation(self, obs: np.ndarray) -> np.ndarray:
        self.history.append(obs)
        if len(self.history) < self.history.maxlen:
            while len(self.history) < self.history.maxlen:
                self.history.appendleft(obs)
        return np.concatenate(list(self.history))

    def run(
        self,
        lane_offset: float,
        lane_heading: float,
        detections: Optional[Sequence[Dict]],
    ) -> Tuple[float, float]:
        blocked = ego_lane_blocked(detections, self.safety_gap_m)
        if self.state == "Follow" and blocked:
            self.state = "Prepare"
        elif self.state == "Prepare":
            if free_on_left(detections):
                self.state = "ChangeLeft"
            elif free_on_right(detections):
                self.state = "ChangeRight"
        elif self.state in ("ChangeLeft", "ChangeRight") and aligned_with_lane(lane_offset, lane_heading):
            self.state = "Follow"

        obs = _build_observation(lane_offset, lane_heading, detections)
        stacked_obs = self._stack_observation(obs) if self.history.maxlen > 1 else obs

        min_center = _distance_in_lane(detections, "center")
        emergency_stop = min_center is not None and min_center < self.stop_distance_m

        if self.policy:
            action = self.policy.act(stacked_obs, deterministic=emergency_stop)
            angle = float(np.clip(action[0], -1.0, 1.0))
            throttle_cmd = float(np.clip(action[1], -1.0, 1.0))
            throttle = ((throttle_cmd + 1.0) / 2.0) * self.max_throttle
        else:
            angle, throttle = _heuristic_action(
                lane_offset, lane_heading, detections, self.max_throttle
            )

        if emergency_stop:
            throttle = 0.0

        if self.state == "ChangeLeft":
            angle = np.clip(angle - 0.15, -1.0, 1.0)
        elif self.state == "ChangeRight":
            angle = np.clip(angle + 0.15, -1.0, 1.0)

        return float(angle), float(np.clip(throttle, 0.0, self.max_throttle))
