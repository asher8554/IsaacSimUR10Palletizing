# guide.src.html의 코드 자리표시자를 실제 소스로 채워 guide.html을 만든다
"""실행(저장소 루트에서, 아무 Python 3.9+): python docs/build_guide.py

자리표시자
    {{code:step01_load_robot.py}}                  파일 전체
    {{code:palletizing/scene.py:build_scene}}      함수·클래스 하나 (클래스 메서드는 Class.method)
    {{code:palletizing/scene.py:ENV_PATH}}         최상위 대입문 하나

가이드에 실리는 코드가 시험한 코드와 달라지지 않게 하려고 이렇게 한다.
"""

import ast
import html
import io
import keyword
import re
import sys
import tokenize
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docs" / "guide.src.html"
OUT = ROOT / "docs" / "guide.html"


def find_node(tree: ast.AST, dotted: str) -> ast.AST:
    node = tree
    for part in dotted.split("."):
        for child in ast.iter_child_nodes(node):
            if getattr(child, "name", None) == part:
                break
            if isinstance(child, (ast.Assign, ast.AnnAssign)):
                targets = child.targets if isinstance(child, ast.Assign) else [child.target]
                if any(getattr(t, "id", None) == part for t in targets):
                    break
        else:
            raise KeyError(dotted)
        node = child
    return node


def extract(path: str, name: str | None) -> str:
    text = (ROOT / path).read_text(encoding="utf-8")
    if name is None:
        return text.rstrip("\n")
    node = find_node(ast.parse(text), name)
    lines = text.splitlines()
    start = min([node.lineno] + [d.lineno for d in getattr(node, "decorator_list", [])])
    block = lines[start - 1 : node.end_lineno]
    indent = len(block[0]) - len(block[0].lstrip())
    return "\n".join(line[indent:] for line in block)


def highlight(code: str) -> str:
    """tokenize로 주석·문자열·키워드·숫자에 색 클래스를 입힌다."""
    out, row, col = [], 1, 0
    lines = code.split("\n")
    previous = None
    try:
        tokens = list(tokenize.generate_tokens(io.StringIO(code + "\n").readline))
    except (tokenize.TokenError, IndentationError):
        return html.escape(code)
    for tok in tokens:
        if tok.type in (tokenize.ENDMARKER, tokenize.INDENT, tokenize.DEDENT, tokenize.NEWLINE, tokenize.NL):
            continue
        (srow, scol), (erow, ecol) = tok.start, tok.end
        while row < srow:  # 토큰 사이의 줄바꿈과 공백을 원문 그대로 옮긴다
            out.append(html.escape(lines[row - 1][col:]) + "\n")
            row, col = row + 1, 0
        out.append(html.escape(lines[row - 1][col:scol]))
        text = html.escape(tok.string)
        cls = None
        if tok.type == tokenize.COMMENT:
            cls = "c"
        elif tok.type == tokenize.STRING or tok.type in (
            getattr(tokenize, "FSTRING_START", -1), getattr(tokenize, "FSTRING_MIDDLE", -1), getattr(tokenize, "FSTRING_END", -1),
        ):  # fmt: skip
            cls = "s"
        elif tok.type == tokenize.NUMBER:
            cls = "n"
        elif tok.type == tokenize.NAME:
            if keyword.iskeyword(tok.string):
                cls = "k"
            elif previous in ("def", "class"):
                cls = "f"
            elif tok.string in ("True", "False", "None", "self", "cls"):
                cls = "b"
        out.append(f'<span class="{cls}">{text}</span>' if cls else text)
        previous = tok.string
        row, col = erow, ecol
    return "".join(out).rstrip("\n")


def render(match: re.Match) -> str:
    path, _, name = match.group(1).partition(":")
    code = extract(path.strip(), name.strip() or None)
    label = path if not name else f"{path} · {name}"
    lines = code.count("\n") + 1
    return (
        f'<figure class="code"><figcaption><span class="file">{html.escape(label)}</span>'
        f'<span class="meta">{lines}줄</span><button type="button" class="copy">복사</button></figcaption>'
        f'<pre tabindex="0"><code>{highlight(code)}</code></pre></figure>'
    )


def main() -> int:
    source = SRC.read_text(encoding="utf-8")
    result = re.sub(r"\{\{code:([^}]+)\}\}", render, source)
    OUT.write_text(result, encoding="utf-8", newline="\n")
    print(f"{OUT.relative_to(ROOT)} 작성: {len(result) / 1024:.0f} KB, 코드 블록 {len(re.findall('{{code:', source))}개")
    return 0


if __name__ == "__main__":
    sys.exit(main())
