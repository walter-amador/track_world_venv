"""
model_loader.py — depth model abstraction for obst_avoid.

Each class wraps one monocular depth model and exposes a common interface:
  .load()                → download weights + build model (called once at startup)
  .infer(bgr) → ndarray  → raw depth map, same H×W as input
  .to_normalized(raw)    → float32 [0,1] where 0=closest, 1=farthest

Supported models (select via 'model_name' ROS parameter):
  midas            — MiDaS (Intel ISL), relative inverse depth
  depth_anything_v2— Depth Anything V2 (HuggingFace), relative inverse depth
  zoe_depth        — ZoeDepth (ISL), metric depth in metres
  depth_pro        — Apple Depth Pro, metric depth in metres (needs separate install)
"""

import abc
import os
import numpy as np


def _pretrust_hub_repo(owner: str, project: str) -> None:
    """
    Write owner_project to torch's trusted_list file so that torch.hub.load
    calls that happen inside third-party code (e.g. MiDaS loading its backbone)
    don't hang waiting for interactive confirmation in a non-TTY context.
    Safe to call multiple times; is idempotent.
    """
    import torch

    hub_dir = torch.hub.get_dir()
    trusted_path = os.path.join(hub_dir, "trusted_list")
    os.makedirs(hub_dir, exist_ok=True)
    entry = f"{owner}_{project}"
    existing: list[str] = []
    if os.path.exists(trusted_path):
        with open(trusted_path) as fh:
            existing = [ln.strip() for ln in fh]
    if entry not in existing:
        with open(trusted_path, "a") as fh:
            fh.write(entry + "\n")


class DepthModel(abc.ABC):
    name: str
    # If True, raw output is metric depth (lower=closer).
    # If False, raw output is disparity/inverse-depth (higher=closer).
    is_metric: bool

    @abc.abstractmethod
    def load(self) -> None:
        """Download weights and initialise the model. Called once at startup."""

    @abc.abstractmethod
    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        """
        Run inference on a BGR uint8 image.
        Returns a float32 H×W array in the model's native units.
        """

    def to_normalized(self, raw: np.ndarray) -> np.ndarray:
        """
        Normalise raw output to float32 [0, 1] where:
          0 → closest (most dangerous)
          1 → farthest (safest)
        Uses 1st–99th percentile clipping to suppress outliers.
        """
        p1 = float(np.percentile(raw, 1))
        p99 = float(np.percentile(raw, 99))
        if p99 - p1 < 1e-6:
            return np.zeros(raw.shape, dtype=np.float32)
        norm = (raw.astype(np.float32) - p1) / (p99 - p1)
        norm = np.clip(norm, 0.0, 1.0)
        if not self.is_metric:
            # disparity: larger value = closer → invert so 0=closest
            norm = 1.0 - norm
        return norm


# ─────────────────────────────────────────────────────────────────────────────
# MiDaS
# ─────────────────────────────────────────────────────────────────────────────
class MiDaSModel(DepthModel):
    """
    MiDaS — Intel ISL monocular depth.
    torch.hub model; weights download automatically on first load (~90 MB small).

    Variants: 'small' (fast, GPU optional), 'hybrid' (DPT+ViT, ~330 MB),
              'large' (best quality, ~470 MB).
    """

    name = "midas"
    is_metric = False

    _VARIANT_MAP = {
        "small": "MiDaS_small",
        "hybrid": "DPT_Hybrid",
        "large": "DPT_Large",
    }

    def __init__(self, variant: str = "small"):
        self._variant_key = self._VARIANT_MAP.get(variant, "MiDaS_small")

    def load(self) -> None:
        import torch
        # MiDaS_small internally calls torch.hub.load("rwightman/gen-efficientnet-pytorch")
        # without trust_repo=True, which hangs non-interactively.  Pre-trust it.
        _pretrust_hub_repo("rwightman", "gen-efficientnet-pytorch")
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model = torch.hub.load(
            "intel-isl/MiDaS", self._variant_key, trust_repo=True
        )
        self._model.to(self._device).eval()

        transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
        if "small" in self._variant_key.lower():
            self._transform = transforms.small_transform
        else:
            self._transform = transforms.dpt_transform

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        import torch
        import cv2

        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        batch = self._transform(rgb).to(self._device)
        with torch.no_grad():
            pred = self._model(batch)
            pred = torch.nn.functional.interpolate(
                pred.unsqueeze(1),
                size=bgr_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()
        return pred.cpu().numpy().astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Depth Anything V2
# ─────────────────────────────────────────────────────────────────────────────
class DepthAnythingV2Model(DepthModel):
    """
    Depth Anything V2 — HuggingFace transformers pipeline.
    Install: pip install transformers torch Pillow

    Variants: 'small' (~98 MB), 'base' (~390 MB), 'large' (~1.3 GB).
    Weights are cached in ~/.cache/huggingface/ on first load.
    """

    name = "depth_anything_v2"
    is_metric = False

    _CHECKPOINTS = {
        "small": "depth-anything/Depth-Anything-V2-Small-hf",
        "base": "depth-anything/Depth-Anything-V2-Base-hf",
        "large": "depth-anything/Depth-Anything-V2-Large-hf",
    }

    def __init__(self, variant: str = "small"):
        self._checkpoint = self._CHECKPOINTS.get(variant, self._CHECKPOINTS["small"])

    def load(self) -> None:
        import torch
        from transformers import pipeline as hf_pipeline

        device = 0 if torch.cuda.is_available() else -1
        self._pipe = hf_pipeline(
            task="depth-estimation",
            model=self._checkpoint,
            device=device,
        )

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        import cv2
        from PIL import Image

        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        result = self._pipe(Image.fromarray(rgb))
        depth = np.array(result["depth"], dtype=np.float32)
        if depth.shape[:2] != bgr_image.shape[:2]:
            depth = cv2.resize(depth, (bgr_image.shape[1], bgr_image.shape[0]))
        return depth


# ─────────────────────────────────────────────────────────────────────────────
# ZoeDepth
# ─────────────────────────────────────────────────────────────────────────────
class ZoeDepthModel(DepthModel):
    """
    ZoeDepth — ISL metric monocular depth.
    torch.hub model; weights download automatically on first load.

    Variants: 'n' (NYU indoor, ~340 MB), 'k' (KITTI outdoor), 'nk' (mixed).
    For indoor/sim use 'n'.  Output is metric depth in metres.
    """

    name = "zoe_depth"
    is_metric = True

    _VARIANT_MAP = {"n": "ZoeD_N", "k": "ZoeD_K", "nk": "ZoeD_NK"}

    def __init__(self, variant: str = "n"):
        self._variant_key = self._VARIANT_MAP.get(variant, "ZoeD_N")

    def load(self) -> None:
        import torch

        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model = torch.hub.load(
            "isl-org/ZoeDepth", self._variant_key, pretrained=True, trust_repo=True
        )
        self._model.to(self._device).eval()

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        import cv2
        from PIL import Image
        import torch

        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb)
        with torch.no_grad():
            depth = self._model.infer_pil(pil_img)
        return np.array(depth, dtype=np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Apple Depth Pro
# ─────────────────────────────────────────────────────────────────────────────
class DepthProModel(DepthModel):
    """
    Apple ML Depth Pro — sharp-boundary metric monocular depth.
    Output: metric depth in metres at full input resolution.

    REQUIRES separate installation (not on PyPI):
      pip install git+https://github.com/apple/ml-depth-pro.git
      # then download weights as described in the repo README

    Camera focal length is hardcoded to match limo_sim (70° HFOV, 640 px wide):
      fx = 640 / (2 * tan(35°)) ≈ 457 px
    """

    name = "depth_pro"
    is_metric = True
    # limo_sim camera: HFOV=70°, W=640 → fx = W / (2*tan(HFOV/2))
    _F_PX = 457.0

    def load(self) -> None:
        try:
            import depth_pro
        except ImportError:
            raise ImportError(
                "depth_pro package not found.\n"
                "Install with:\n"
                "  pip install git+https://github.com/apple/ml-depth-pro.git\n"
                "Then download weights per the ml-depth-pro README."
            )
        import torch

        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model, self._transform = depth_pro.create_model_and_transforms(
            device=self._device
        )
        self._model.eval()
        self._depth_pro = depth_pro

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        import cv2
        from PIL import Image
        import torch

        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        image = self._transform(Image.fromarray(rgb))
        with torch.no_grad():
            prediction = self._model.infer(image, f_px=self._F_PX)
        depth = prediction["depth"].squeeze().cpu().numpy()
        if depth.shape[:2] != bgr_image.shape[:2]:
            depth = cv2.resize(depth, (bgr_image.shape[1], bgr_image.shape[0]))
        return depth.astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Factory
# ─────────────────────────────────────────────────────────────────────────────
_REGISTRY = {
    "midas": MiDaSModel,
    "depth_anything_v2": DepthAnythingV2Model,
    "zoe_depth": ZoeDepthModel,
    "depth_pro": DepthProModel,
}


def build_model(model_name: str, variant: str = "small") -> DepthModel:
    """
    Instantiate (but do NOT load) the requested model.
    Call .load() after construction to download weights.

    Args:
        model_name: one of 'midas', 'depth_anything_v2', 'zoe_depth', 'depth_pro'
        variant:    model-specific size hint ('small', 'base', 'large', 'n', …)
    """
    cls = _REGISTRY.get(model_name)
    if cls is None:
        raise ValueError(
            f"Unknown model '{model_name}'. "
            f"Available: {list(_REGISTRY.keys())}"
        )
    return cls(variant=variant)
