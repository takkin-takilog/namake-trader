import pandas as pd
from pathlib import Path


def save_df_csv(
    filepath: str,
    df: pd.DataFrame,
    index: bool = True,
    date_format: str | None = None,
) -> None:
    p = Path(filepath)
    p.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(filepath, index=index, date_format=date_format)


def load_df_csv(filepath: str) -> pd.DataFrame:
    p = Path(filepath)
    if not p.exists():
        raise FileNotFoundError("Could not found [" + str(filepath) + "]")

    return pd.read_csv(filepath)
