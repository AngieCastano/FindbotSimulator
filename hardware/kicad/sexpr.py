"""Lector y escritor mínimo de expresiones S (formato de archivos de KiCad).

Las cadenas entre comillas se representan con la clase Q para conservar las comillas al
volver a escribir; los átomos sin comillas quedan como str.
"""


class Q(str):
    """Cadena que se escribe entre comillas."""


def parse(text):
    """Convierte el texto de un archivo KiCad en listas anidadas."""
    stack = [[]]
    i, n = 0, len(text)
    while i < n:
        ch = text[i]
        if ch == '(':
            stack.append([])
            i += 1
        elif ch == ')':
            done = stack.pop()
            stack[-1].append(done)
            i += 1
        elif ch == '"':
            j = i + 1
            buf = []
            while text[j] != '"':
                if text[j] == '\\':
                    j += 1
                    buf.append({'n': '\n', 't': '\t'}.get(text[j], text[j]))
                else:
                    buf.append(text[j])
                j += 1
            stack[-1].append(Q(''.join(buf)))
            i = j + 1
        elif ch.isspace():
            i += 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()"':
                j += 1
            stack[-1].append(text[i:j])
            i = j
    return stack[0][0]


def _atom(x):
    if isinstance(x, Q):
        return '"' + x.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'
    if isinstance(x, float):
        return f'{x:.4f}'.rstrip('0').rstrip('.') if x != int(x) else str(int(x))
    return str(x)


def dump(node, indent=0):
    """Escribe listas anidadas como expresión S con sangría de tabuladores."""
    if not isinstance(node, list):
        return _atom(node)
    simple = all(not isinstance(x, list) for x in node)
    if simple:
        return '(' + ' '.join(_atom(x) for x in node) + ')'
    pad = '\t' * (indent + 1)
    head = [_atom(x) for x in node if not isinstance(x, list)]
    out = '(' + ' '.join(head)
    for x in node:
        if isinstance(x, list):
            out += '\n' + pad + dump(x, indent + 1)
    return out + '\n' + '\t' * indent + ')'


def find(node, key):
    """Primer hijo cuya cabecera es key (o None)."""
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def find_all(node, key):
    """Todos los hijos cuya cabecera es key."""
    return [x for x in node if isinstance(x, list) and x and x[0] == key]
