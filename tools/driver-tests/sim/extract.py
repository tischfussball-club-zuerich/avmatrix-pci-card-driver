# Extract named functions verbatim from the driver source (brace matching,
# skipping #if 0 regions so braces inside them don't count).
import re, sys

src = open(sys.argv[1]).read().split("\n")
names = sys.argv[2:]

def live_lines():
    """yield (index, line, live) where live is False inside #if 0 blocks"""
    depth = 0          # nesting depth inside an #if 0 block
    for i, l in enumerate(src):
        s = l.strip()
        if depth:
            if re.match(r"#\s*if", s):
                depth += 1
            elif re.match(r"#\s*endif", s):
                depth -= 1
            elif re.match(r"#\s*else", s) and depth == 1:
                depth = 0
            yield i, l, False
            continue
        if re.match(r"#\s*if\s+0\b", s):
            depth = 1
            yield i, l, False
            continue
        yield i, l, True

lines = list(live_lines())

def extract(name):
    pat = re.compile(r"^(static\s+)?(inline\s+)?[A-Za-z_][\w\s\*]*?\b" + re.escape(name) + r"\s*\(")
    for k, (i, l, live) in enumerate(lines):
        if live and pat.match(l) and not l.rstrip().endswith(";"):
            out, depth, started = [], 0, False
            for j, l2, live2 in lines[k:]:
                out.append(l2)
                if live2:
                    code = re.sub(r"//.*", "", l2)
                    code = re.sub(r'"(\\.|[^"\\])*"', '""', code)
                    code = re.sub(r"'(\\.|[^'\\])'", "''", code)
                    depth += code.count("{") - code.count("}")
                    if "{" in code:
                        started = True
                    if started and depth == 0:
                        return "\n".join(out)
            raise SystemExit("unterminated " + name)
    raise SystemExit("not found: " + name)

for n in names:
    print("/* ---- extracted verbatim: %s ---- */" % n)
    print(extract(n))
    print()
