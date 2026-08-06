# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Scene construction: terrain, boulders, lighting.

boulders.py is pure numpy and is tested in CI. The rest need Isaac and import
it lazily inside their build() functions.
"""
