"""
Zero-position calibration visualization.
Left: mechanism frame. Right: cart frame (standard +Y up).
Motor O is mounted on the cart body, and P7 is the wheel hub under the cart.
"""
import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from matplotlib.patches import Circle, Rectangle
from .mechanism import default_params
from .kinematics import solve_linkage

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PIC_DIR = PROJECT_ROOT / "pic"
OUTPUT_PATH = PIC_DIR / "zero_calib.png"

theta_a = np.deg2rad(-72.4)
theta_b = np.deg2rad(80.0)
phi     = np.deg2rad(13.9)

params = default_params()
res = solve_linkage(theta_a, theta_b, params)
assert res is not None

# The linkage drawing in cart coordinates must be flipped relative to the
# mechanism-frame drawing.  Without the +pi term the wheel hub ends up above
# the motor in the cart frame.
cart_angle = phi + np.pi
c, s = np.cos(cart_angle), np.sin(cart_angle)
def to_cart(v):
    return np.array([c*v[0] - s*v[1], s*v[0] + c*v[1]])

# Cart frame convention:
#   +X is forward, +Y is up.
#   The cart body is a horizontal rectangle.
#   P7 is the wheel hub; the wheel circle sits below the cart and touches
#   the ground at Y = 0.
wheel_radius = 30.0
cart_body_w = 180.0
cart_body_h = 24.0
p7_rel_cart = to_cart(res['P7'])
wheel_cart = np.array([p7_rel_cart[0], wheel_radius])
O_cart = wheel_cart - p7_rel_cart
cart_body_y = O_cart[1] - cart_body_h * 0.5
cart_body_x = O_cart[0] - cart_body_w * 0.5

fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(14, 7))
bar_colors = ['#e74c3c','#e74c3c','#3498db','#2ecc71','#f39c12','#f39c12',
              '#9b59b6','#1abc9c','#1abc9c']
pairs = [('O','P1'),('P1','P2'),('O','P3'),('P3','P4'),
         ('P4','P1'),('P1','P5'),('P5','P6'),('P6','P2'),('P2','P7')]

# ===== LEFT: mechanism frame =====
for (ka,kb), clr in zip(pairs, bar_colors):
    a, b = res[ka], res[kb]
    ax_l.plot([a[0],b[0]], [a[1],b[1]], '-', color=clr, lw=2.5)

ax_l.scatter(0, 0, c='red', s=100, zorder=5)
ax_l.scatter(*res['P7'], c='green', s=120, zorder=5)
ax_l.annotate('O', (0,0), textcoords='offset points', xytext=(6,6), fontsize=10, fontweight='bold')
ax_l.annotate(f'P7 ({res["P7"][0]:.0f},{res["P7"][1]:.0f})', res['P7'],
              textcoords='offset points', xytext=(10,-15), fontsize=9, color='green')
p3 = res['P3']; d3 = p3 / np.linalg.norm(p3) * 65
ax_l.arrow(0, 0, d3[0], d3[1], head_width=3, head_length=5, fc='#3498db', ec='#3498db', alpha=0.4, lw=2)
ax_l.annotate(f'bar_b={np.rad2deg(theta_b):.1f}$^\\circ$', d3*0.75, fontsize=8, color='#3498db')
ax_l.set_title(f'Mechanism Frame\n$\\theta_a$={np.rad2deg(theta_a):.1f}$^\\circ$  '
               f'$\\theta_b$={np.rad2deg(theta_b):.1f}$^\\circ$', fontsize=12)
ax_l.set_xlabel('X [mm]'); ax_l.set_ylabel('Y [mm]')
ax_l.set_aspect('equal'); ax_l.grid(True, alpha=0.3)

# ===== RIGHT: cart frame (horizontal cart body, wheel under cart) =====
for (ka,kb), clr in zip(pairs, bar_colors):
    a_c = to_cart(res[ka]) + O_cart
    b_c = to_cart(res[kb]) + O_cart
    ax_r.plot([a_c[0],b_c[0]], [a_c[1],b_c[1]], '-', color=clr, lw=2.5)

# Cart body and wheel
cart_body = Rectangle((cart_body_x, cart_body_y), cart_body_w, cart_body_h,
                      facecolor='#b0b7c3', edgecolor='#505866',
                      lw=2.0, alpha=0.45, zorder=0)
wheel = Circle(wheel_cart, wheel_radius, facecolor='#3f4650',
               edgecolor='#15191f', lw=2.0, alpha=0.9, zorder=1)
hub = Circle(wheel_cart, wheel_radius * 0.18, facecolor='#d7dde7',
             edgecolor='#15191f', lw=1.0, zorder=6)
ax_r.add_patch(cart_body)
ax_r.add_patch(wheel)
ax_r.add_patch(hub)

ax_r.scatter(*O_cart, c='red', s=100, zorder=5)
ax_r.scatter(*wheel_cart, c='green', s=120, zorder=5)
ax_r.annotate(f'O (motor)\n({O_cart[0]:.0f}, {O_cart[1]:.0f})',
              O_cart, textcoords='offset points',
              xytext=(6,-15), fontsize=9, fontweight='bold')
ax_r.annotate(f'P7 (wheel hub)\n({wheel_cart[0]:.0f}, {wheel_cart[1]:.0f})', wheel_cart,
              textcoords='offset points', xytext=(10,-15), fontsize=9, color='green')

# Ground at Y=0
ax_r.axhline(0, color='#8B4513', lw=3, alpha=0.7)
ax_r.annotate('GROUND', (wheel_cart[0]/2, 2), fontsize=11, color='#8B4513',
              ha='center', fontweight='bold')

# bar_b
p3c = to_cart(res['P3']) + O_cart
d3d = (p3c - O_cart) / np.linalg.norm(p3c - O_cart) * 65
ax_r.arrow(*O_cart, d3d[0], d3d[1], head_width=3, head_length=5,
           fc='#3498db', ec='#3498db', alpha=0.4, lw=2)
ax_r.annotate(f'bar_b', O_cart + d3d*0.8, fontsize=8, color='#3498db')

ax_r.set_title(f'Cart Frame\n$\\phi$ = {np.rad2deg(phi):.1f}$^\\circ$  '
               f'(body horizontal, wheel under body)', fontsize=12)
ax_r.set_xlabel('X [mm] (forward)')
ax_r.set_ylabel('Y [mm] ($\\uparrow$ up)')
ax_r.set_aspect('equal'); ax_r.grid(True, alpha=0.3)

PIC_DIR.mkdir(exist_ok=True)
plt.tight_layout()
plt.savefig(OUTPUT_PATH, dpi=150, bbox_inches='tight')
print(f'Saved: {OUTPUT_PATH}')
print(f'theta_a = {np.rad2deg(theta_a):.1f} deg')
print(f'theta_b = {np.rad2deg(theta_b):.1f} deg')
print(f'phi     = {np.rad2deg(phi):.1f} deg')
print(f'cart drawing angle = phi + 180 deg = {np.rad2deg(cart_angle):.1f} deg')
print(f'P7_cart (rel to O) = ({p7_rel_cart[0]:.1f}, {p7_rel_cart[1]:.1f})')
print(f'wheel radius = {wheel_radius:.0f} mm')
print(f'O_motor in cart frame  = ({O_cart[0]:.0f}, {O_cart[1]:.0f})')
print(f'P7_wheel hub in cart frame = ({wheel_cart[0]:.0f}, {wheel_cart[1]:.0f})')
