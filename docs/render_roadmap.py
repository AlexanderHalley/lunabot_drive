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

HEAD = 246
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


# The left column is everything that exists today, held there by the rank=same
# group in the .dot. Find its right edge so the built / not-written boundary can
# be drawn as an actual line rather than left for the reader to infer.
def right_edge_of(node_ids):
    edge = 0.0
    for nid in node_ids:
        m_node = re.search(
            r'<title>' + re.escape(nid) + r'</title>\s*<path[^>]*\sd="([^"]*)"',
            raw,
        )
        if m_node is None:  # a node was renamed in the .dot but not here
            continue
        xs = [float(p.split(',')[0]) for p in re.findall(r'(-?[\d.]+,-?[\d.]+)', m_node.group(1))]
        edge = max([edge] + xs)
    return edge + 4  # graphviz translates the whole graph by 4


BUILT = [
    'b_ws',
    'b_ctrl',
    'b_can',
    'b_mux',
    'b_cam',
    'b_nav',
    'b_ci',
    'b_docs',
    'b_check',
    'b_probe',
    'b_urdf',
    'b_rtab',
    'b_boul',
    'b_cuv',
    'b_ekf',
    'b_isaac',
    'gpu',
    'chassis',
    'encoders',
]
DIVIDER = right_edge_of(BUILT) + 30
BAND = 42  # room above the graph for the two column captions
body = f'<g transform="translate(0,{BAND})">{body}</g>'

o = []
a = o.append
a(
    f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
    f'width="{W:.0f}" height="{H + BAND + HEAD + FOOT:.0f}" '
    f'viewBox="0 {-HEAD} {W:.0f} {H + BAND + HEAD + FOOT:.0f}" '
    f'font-family="{SANS}">'
)
a('<title>Lunabot 2027 - software roadmap</title>')
a(
    f'<rect x="0" y="{-HEAD}" width="{W:.0f}" '
    f'height="{H + BAND + HEAD + FOOT:.0f}" fill="{SURFACE}"/>'
)

# ------------------------------------------------------------------ header --
a(
    f'<text x="34" y="{-HEAD + 52}" font-size="34" font-weight="700" fill="{INK}">'
    f'Lunabot &#8212; the software road to Lunabotics 2027</text>'
)
a(
    f'<text x="34" y="{-HEAD + 82}" font-size="16" fill="{INK2}">'
    f'Everything the software has to reach between today and competition week. Read it '
    f'left to right; an arrow means the thing it points at cannot start until the '
    f'thing behind it is done.</text>'
)
a(
    f'<text x="34" y="{-HEAD + 105}" font-size="13.5" fill="{MUTED}">'
    f'Branch <tspan font-family="{MONO}">develop-2027</tspan> as of 10 August 2026 &#183; '
    f'the stack builds and drives on mock hardware; Isaac has never been run '
    f'and no rover has ever followed a path.</text>'
)

# ------------------------------------------------------------------ legend --
# Two independent channels, so they get two rows. Conflating them is exactly
# the misreading this legend exists to prevent.
FILLS = [
    ('#d5e8d4', 'done', 'built and trusted'),
    ('#fff2cc', 'in progress', 'built, but known to be provisional'),
    ('#ffffff', 'not started', 'nothing written yet'),
]
KINDS = [
    ('#9a9a94', 1.4, False, 'work', 'a job somebody does'),
    (
        '#d6b656',
        2.6,
        False,
        'GATE',
        'a proof point -- nothing behind it is real work until it clears',
    ),
    ('#b85450', 2.2, False, 'YOU', 'a decision no software closes'),
    ('#6c8ebf', 2.2, False, 'DELIVER', 'an artifact that leaves the team'),
    ('#9673a6', 2.2, False, 'date', 'fixed, not ours to move'),
    ('#9a9a94', 1.4, True, 'WAITING', 'somebody else has to hand it to us'),
]

ly = -HEAD + 128
a(
    f'<line x1="34" y1="{ly - 16}" x2="{W - 34:.0f}" y2="{ly - 16}" '
    f'stroke="{RULE}" stroke-width="1"/>'
)


def legend_caption(x, y, text):
    a(
        f'<text x="{x}" y="{y}" font-size="11" font-weight="700" fill="{MUTED}" '
        f'letter-spacing="0.7">{text}</text>'
    )


def legend_entry(x, y, name, desc):
    a(f'<text x="{x}" y="{y}" font-size="12.5" font-weight="700" fill="{INK}">{name}</text>')
    a(f'<text x="{x}" y="{y + 15}" font-size="11.5" fill="{MUTED}">{desc}</text>')
    return 48 + max(len(desc) * 6.3, len(name) * 8) + 30


legend_caption(34, ly, 'FILL = PROGRESS')
lx = 176
for fill, name, desc in FILLS:
    a(
        f'<rect x="{lx}" y="{ly - 12}" width="30" height="19" rx="5" fill="{fill}" '
        f'stroke="#9a9a94" stroke-width="1.4"/>'
    )
    lx += legend_entry(lx + 38, ly - 3, name, desc)

legend_caption(34, ly + 40, 'BORDER = KIND')
lx = 176
for stroke, pw, dashed, name, desc in KINDS:
    dash = ' stroke-dasharray="4 3"' if dashed else ''
    a(
        f'<rect x="{lx}" y="{ly + 28}" width="30" height="19" rx="5" fill="#ffffff" '
        f'stroke="{stroke}" stroke-width="{pw}"{dash}/>'
    )
    lx += legend_entry(lx + 38, ly + 37, name, desc)

# ------------------------------------------------------- built / not built --
# A tinted band behind the left column and a rule at its edge. Everything to
# the left of the rule exists in the tree today; everything to the right of it
# is the season's work.
a(
    f'<rect x="0" y="0" width="{DIVIDER:.0f}" height="{H + BAND:.0f}" '
    f'fill="#d5e8d4" fill-opacity="0.22"/>'
)
a(
    f'<line x1="{DIVIDER:.0f}" y1="0" x2="{DIVIDER:.0f}" y2="{H + BAND:.0f}" '
    f'stroke="#82b366" stroke-width="1.5" stroke-dasharray="7 5"/>'
)
a(
    '<text x="34" y="26" font-size="14" font-weight="700" fill="#4a7a3a" '
    'letter-spacing="0.8">ALREADY BUILT</text>'
)
a(
    f'<text x="{DIVIDER + 22:.0f}" y="26" font-size="14" font-weight="700" '
    f'fill="{MUTED}" letter-spacing="0.8">EVERYTHING STILL TO DO</text>'
)

# --------------------------------------------------------------- the graph --
a(body)

# ------------------------------------------------------------------ footer --
fy = H + BAND + 34
a(
    f'<line x1="34" y1="{H + BAND + 10:.0f}" x2="{W - 34:.0f}" y2="{H + BAND + 10:.0f}" '
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
print(f'wrote {OUT}  {W:.0f}x{H + BAND + HEAD + FOOT:.0f}')
