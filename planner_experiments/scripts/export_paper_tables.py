#!/usr/bin/env python3
"""Export existing real summary CSV files to Markdown and LaTeX."""

import argparse
import pathlib

try:
    import pandas as pd
except ImportError as exception:
    raise SystemExit("pandas is required: " + str(exception))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_directory")
    args = parser.parse_args()
    root = pathlib.Path(args.result_directory).resolve()
    if not root.joinpath("manifest.json").exists():
        candidates = sorted(p for p in root.iterdir()
                            if p.is_dir() and p.joinpath("manifest.json").exists())
        if not candidates:
            raise SystemExit("manifest not found")
        root = candidates[-1]
    summary = root / "summary"
    outputs = []

    def cell(value) -> str:
        if pd.isna(value):
            return ""
        if isinstance(value, float):
            return f"{value:.6g}"
        return str(value)

    def markdown_table(frame) -> str:
        columns = [str(column) for column in frame.columns]
        lines = ["| " + " | ".join(columns) + " |",
                 "| " + " | ".join("---" for _ in columns) + " |"]
        for row in frame.itertuples(index=False, name=None):
            lines.append("| " + " | ".join(cell(value).replace("|", "\\|")
                                              for value in row) + " |")
        return "\n".join(lines) + "\n"

    def latex_escape(value: str) -> str:
        for source, target in [("\\", r"\textbackslash{}"), ("_", r"\_"),
                               ("%", r"\%"), ("&", r"\&"), ("#", r"\#")]:
            value = value.replace(source, target)
        return value

    def latex_table(frame) -> str:
        columns = [latex_escape(str(column)) for column in frame.columns]
        alignment = "l" * len(columns)
        lines = [f"\\begin{{tabular}}{{{alignment}}}", r"\toprule",
                 " & ".join(columns) + r" \\", r"\midrule"]
        for row in frame.itertuples(index=False, name=None):
            lines.append(" & ".join(latex_escape(cell(value)) for value in row) + r" \\")
        lines.extend([r"\bottomrule", r"\end{tabular}"])
        return "\n".join(lines) + "\n"

    for csv_path in sorted(summary.glob("*.csv")):
        frame = pd.read_csv(csv_path)
        markdown = csv_path.with_suffix(".md")
        latex = csv_path.with_suffix(".tex")
        markdown.write_text(markdown_table(frame), encoding="utf-8")
        latex.write_text(latex_table(frame), encoding="utf-8")
        outputs.extend([str(markdown), str(latex)])
    print("\n".join(outputs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
