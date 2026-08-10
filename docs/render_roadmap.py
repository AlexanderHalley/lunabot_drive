#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Render docs/roadmap-2027.dot into a self-contained SVG.

    sudo apt-get install graphviz
    python3 docs/render_roadmap.py

`dot` lays the graph out; everything here is the page around it -- the title
block, the node-type legend and the footer, none of which graphviz can express.
The graph itself is edited in the .dot file, not here.
"""

import re
import subprocess
from pathlib import Path

SRC = str(Path(__file__).with_name('roadmap-2027.dot'))
OUT = str(Path(__file__).with_name('roadmap-2027.svg'))

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#898781'
RULE = '#dcdbd4'

HEAD = 206
FOOT = 104

SANS = 'system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif'
MONO = 'ui-monospace,SFMono-Regular,Menlo,Consolas,monospace'

raw = subprocess.run(['dot', '-Tsvg', SRC], capture_output=True, text=True, check=True).stdout

m = re.search(r'viewBox="0\.00 0\.00 ([\d.]+) ([\d.]+)"', raw)
W, H = float(m.group(1)), float(m.group(2))

body = raw[raw.index('<svg ') :]
body = body[body.index('>') + 1 :]
body = body.replace('</svg>', '')

# graphviz paints its own background over the graph area only; drop it so the
# page background shows through uniformly.
body = re.sub(r'<polygon fill="#fcfcfb"[^/]*/>', '', body, count=1)

body = body.replace('font-family="Helvetica,sans-Serif"', f'font-family="{SANS}"')
body = body.replace('font-family="Courier,monospace"', f'font-family="{MONO}"')
body = body.replace('font-family="Times,serif"', f'font-family="{SANS}"')

o = []
a = o.append
a(
    f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
    f'width="{W:.0f}" height="{H + HEAD + FOOT:.0f}" '
    f'viewBox="0 {-HEAD} {W:.0f} {H + HEAD + FOOT:.0f}" '
    f'font-family="{SANS}">'
)
a('<title>Lunabot 2027 - software roadmap</title>')
a(f'<rect x="0" y="{-HEAD}" width="{W:.0f}" height="{H + HEAD + FOOT:.0f}" fill="{SURFACE}"/>')

# ------------------------------------------------------------------ header --
a(
    f'<text x="34" y="{-HEAD + 52}" font-size="34" font-weight="700" fill="{INK}">'
    f'Lunabot &#8212; the software road to Lunabotics 2027</text>'
)
a(
    f'<text x="34" y="{-HEAD + 82}" font-size="16" fill="{INK2}">'
    f'Every piece of software that has to exist between today and competition week. '
    f'Read it left to right; an arrow means the thing it points at cannot start '
    f'until the thing behind it is done.</text>'
)
a(
    f'<text x="34" y="{-HEAD + 105}" font-size="13.5" fill="{MUTED}">'
    f'Branch <tspan font-family="{MONO}">develop-2027</tspan> as of 10 August 2026 &#183; '
    f'the stack builds and drives on mock hardware; Isaac has never been run '
    f'and no rover has ever followed a path.</text>'
)

# ------------------------------------------------------------------ legend --
LEG = [
    ('#d5e8d4', '#82b366', 1.6, 'work', 'a job somebody does'),
    ('#d5e8d4', '#d6b656', 2.6, 'GATE', 'proof that unblocks what follows'),
    ('#fff2cc', '#d6b656', 2.0, 'YOU', 'only a human closes it'),
    ('#dae8fc', '#6c8ebf', 2.0, 'DELIVER', 'an artifact that leaves the team'),
    ('#e1d5e7', '#9673a6', 2.0, 'date', 'fixed, not ours to move'),
    ('#f5f5f5', '#909090', 1.6, 'input', 'state, or something we wait on'),
]
lx = 34
ly = -HEAD + 140
a(
    f'<line x1="34" y1="{ly - 14}" x2="{W - 34:.0f}" y2="{ly - 14}" '
    f'stroke="{RULE}" stroke-width="1"/>'
)
for fill, stroke, pw, name, desc in LEG:
    a(
        f'<rect x="{lx}" y="{ly + 4}" width="30" height="19" rx="5" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{pw}"/>'
    )
    a(
        f'<text x="{lx + 38}" y="{ly + 13}" font-size="12.5" font-weight="700" '
        f'fill="{INK}">{name}</text>'
    )
    a(f'<text x="{lx + 38}" y="{ly + 28}" font-size="11.5" fill="{MUTED}">{desc}</text>')
    lx += 48 + max(len(desc) * 6.3, len(name) * 8) + 34

# --------------------------------------------------------------- the graph --
a(body)

# ------------------------------------------------------------------ footer --
fy = H + 34
a(
    f'<line x1="34" y1="{H + 10:.0f}" x2="{W - 34:.0f}" y2="{H + 10:.0f}" '
    f'stroke="{RULE}" stroke-width="1"/>'
)
a(
    f'<text x="34" y="{fy:.0f}" font-size="14" font-weight="700" fill="{INK}">'
    f'The three that decide the season</text>'
)
a(
    f'<text x="34" y="{fy + 22:.0f}" font-size="13" fill="{INK2}">'
    f'<tspan font-weight="700">1.</tspan> Isaac has never run. Until '
    f'<tspan font-family="{MONO}">probe_isaac_api.sh</tspan> clears, every '
    f'simulation task behind it is a guess. &#160;&#160;&#160;'
    f'<tspan font-weight="700">2.</tspan> The compute choice gates cuVSLAM, and '
    f'cuVSLAM is the only SLAM path with a GPU story. &#160;&#160;&#160;'
    f'<tspan font-weight="700">3.</tspan> There is no mission layer at all &#8212; '
    f'<tspan font-family="{MONO}">lunabot_mission</tspan> is a package that does '
    f'not exist yet, and autonomy is where the points are.</text>'
)
a(
    f'<text x="34" y="{fy + 45:.0f}" font-size="12" fill="{MUTED}">'
    f'The in-repo work queue is every <tspan font-family="{MONO}">PLACEHOLDER</tspan>, '
    f'<tspan font-family="{MONO}">VERIFY</tspan> and '
    f'<tspan font-family="{MONO}">TODO(2027)</tspan> marker &#8212; '
    f'33 files carry one today. Milestone dates are the published NASA Lunabotics 2027 schedule; '
    f'competition week is May 2027 at the Center for Space Education, KSC Visitor '
    f'Complex, exact dates not yet released.</text>'
)
a(
    f'<text x="34" y="{fy + 64:.0f}" font-size="12" fill="{MUTED}">'
    f'Confirm all of it against the guidebook when it lands on 3 September 2026 '
    f'&#8212; the arena, '
    f'the obstacle spec and the autonomy scoring tiers are placeholders until then.</text>'
)

a('</svg>')

with open(OUT, 'w') as f:
    f.write('\n'.join(o) + '\n')
print(f'wrote {OUT}  {W:.0f}x{H + HEAD + FOOT:.0f}')
