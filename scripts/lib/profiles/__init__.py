import importlib
import os


def get_profile():
    name = os.environ.get("ANIME_PROFILE", "dummy")
    return importlib.import_module(f"lib.profiles.{name}")
