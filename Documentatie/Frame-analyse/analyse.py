"""
Frame-analyse Q-Dat Systems -- differentieel aangedreven autonome robotauto.

Onderbouwt drie ontwerpkeuzes:
  * de spoorbreedte b  (hart-op-hart afstand tussen de twee aangedreven wielen)
  * de naloopafstand d (langsafstand van de aandrijfas naar het contactpunt
    van het vrijloopwieltje achter)
  * 2 aangedreven wielen + vrijloopwiel versus 4 aangedreven wielen

Assenstelsel: oorsprong in het midden van de aandrijfas op vloerniveau,
x naar voren, y naar links, z omhoog.  Lengtes in mm, krachten in N, massa in kg.

Uitvoer: figuren/*.pdf + *.png, massastaat.tex en resultaten.tex.

Draaien met de Python die bij Blender 5.2 zit (deze machine heeft geen eigen Python):
  PYTHONPATH=<scratch>/pylibs "C:/Program Files/Blender Foundation/Blender 5.2/5.2/python/bin/python.exe" analyse.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figuren")
os.makedirs(FIG, exist_ok=True)

plt.rcParams.update({
    "font.size": 8.5, "axes.titlesize": 9.5, "axes.labelsize": 8.5,
    "legend.fontsize": 7.0, "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.5,
    "figure.dpi": 160, "savefig.bbox": "tight",
})

C = dict(blue="#2f6fb2", orange="#e07b39", green="#3d8f5b", red="#c0392b",
         purple="#7d5ba6", teal="#2a8a8a", grey="#6b6b6b", light="#dcdcdc")

out = {}


def rec(name, value, fmt="{:.1f}"):
    out[name] = fmt.format(value) if isinstance(value, (int, float, np.floating)) else str(value)
    return value


def save(fig, stem):
    fig.savefig(os.path.join(FIG, stem + ".pdf"))
    fig.savefig(os.path.join(FIG, stem + ".png"), dpi=150)
    plt.close(fig)


# ==========================================================================
# 1. Vaste gegevens
# ==========================================================================
g_acc = 9.81

# --- uit de CAD gemeten (Hardware/3D Files/Parts/SCR_Gear_Motor_Wheel_v1.step)
D_wheel = 64.0          # wieldiameter
r_w = D_wheel / 2
w_tyre = 25.9           # bandbreedte
t_gb = 22.6             # dikte tandwielkast, axiale richting
L_mot = 70.0            # totale motorlengte
h_mot = 22.0            # hoogte motorhuis
x_mot_front = 18.0      # motor steekt voor de aandrijfas uit
x_mot_rear = L_mot - x_mot_front
e_wheel = 14.0          # hart band buiten de zijwand van de tandwielkast

# --- printplaten: breedte (y) x lengte (x) --------------------------------
PCB = {"xplained": (60.0, 75.0),    # CAD-meting van de PCB
       "breadboard": (55.0, 84.0),  # half-size, met voedingsrails
       "l298": (43.0, 43.0),
       "accu": (58.0, 61.0)}        # platte 4xAA houder, vier cellen naast elkaar

# --- vrijloopwieltje (opgemeten aan het echte onderdeel) ------------------
cast_hole_x, cast_hole_y = 30.0, 25.0   # gatenpatroon: 30 langs, 25 dwars
cast_width = 30.0
r_caster_env = 15.0                      # omhullende straal in bovenaanzicht

# --- bedrijfsaannames ------------------------------------------------------
mu = 0.80           # band op gladde vloer
mu_c = 0.20         # zijdelings wegschuiven vrijloopwieltje
C_rr = 0.030        # rolweerstand
tau_stall6 = 0.070  # blokkeerkoppel aan de uitgaande as bij 6 V [Nm]
I_stall6 = 1.10     # blokkeerstroom per motor bij 6 V [A]
I_free = 0.12       # onbelaste stroom per motor, nauwelijks spanningsafhankelijk [A]
n0_rpm = 200.0      # onbelast toerental van de uitgaande as bij 6 V
U_ref = 6.0         # spanning waarbij bovenstaande datablad-waarden gelden [V]

# voeding: vier AA-cellen in serie, via de L298
U_pack = 6.0        # nominale pakspanning, alkaline 4 x 1,5 V [V]
R_cell = 0.15       # inwendige weerstand per AA-cel [ohm]
V_d0, V_d1 = 1.2, 0.6   # spanningsval L298: V = V_d0 + V_d1 * I  [V]

# --- werkelijke klemspanning en werkpunt van de motor ----------------------
# De motor krijgt niet de pakspanning: de cellen zakken in en de L298 slikt een
# vaste val. Alle motorconstanten schalen lineair met de klemspanning, dus dit
# is een vaste-puntiteratie op U_m. Het werkpunt u* volgt uit d(eta)/du = 0.
omega0_ref = n0_rpm * 2 * np.pi / 60.0
R_pack = 4 * R_cell
U_m = U_pack
for _ in range(200):
    s_v = U_m / U_ref
    tau_stall, omega0, I_stall = tau_stall6 * s_v, omega0_ref * s_v, I_stall6 * s_v
    k_duty = (np.sqrt(I_free * I_stall) - I_free) / (I_stall - I_free)
    I_cont = I_free + (I_stall - I_free) * k_duty
    U_m += 0.4 * ((U_pack - 2 * I_cont * R_pack - (V_d0 + V_d1 * I_cont)) - U_m)

tau_cont = k_duty * tau_stall
omega_cont = omega0 * (1 - k_duty)
eta_cont = tau_cont * omega_cont / (U_m * I_cont)
F_cont = 2 * tau_cont / (r_w / 1000.0)
N_axle = 20         # pulsen per wielomwenteling, schijf op de wielas
N_shaft = 20 * 48   # pulsen per wielomwenteling, schijf op de motoras
eps_b = 1.5         # onzekerheid effectieve spoorbreedte [mm]
Ed = 0.01           # relatief verschil wieldiameter links/rechts

# --- ontwerpkeuze ----------------------------------------------------------
b_opt = 98.0
d_opt = 75.0
wall = 3.0
x_front_deck = 54.0     # hoofddek loopt tot hier voor de aandrijfas
x_front_sens = 70.0     # sensorbeugel loopt tot hier

# ==========================================================================
# 2. Pakketrelaties: van motorafstand naar spoorbreedte
# ==========================================================================
def gap_from_b(b):
    return b - 2 * (t_gb + e_wheel)


def frame_width_wheel(b):
    """Framebreedte naast de banden: hooguit gelijk met de zijkant van de kasten."""
    return gap_from_b(b) + 2 * t_gb


def frame_width(b, s=wall):
    """Framebreedte voor en achter de banden, waar wel een wand naast mag."""
    return frame_width_wheel(b) + 2 * s


def total_width(b):
    return b + w_tyre


b_min_pack = 2 * (t_gb + e_wheel)
rec("bMinPack", b_min_pack)
rec("gapOpt", gap_from_b(b_opt))
rec("frameWOpt", frame_width(b_opt))
rec("bfWheel", frame_width_wheel(b_opt))
rec("clearTyre", (b_opt - w_tyre) / 2 - frame_width_wheel(b_opt) / 2, "{:.2f}")
rec("totalWOpt", total_width(b_opt))
rec("WuOpt", frame_width_wheel(b_opt) - 6.0, "{:.0f}")

# ==========================================================================
# 3. Massastaat -> totale massa, zwaartepunt
# ==========================================================================
# (naam, massa kg, x mm, z mm, opmerking)
MASS = [
    ("2 wielen",                    0.060,   0.0, 32.0, "op de aandrijfas"),
    ("2 motoren",                   0.060, -17.0, 32.0, "huis 70 mm, 18 mm voor de as"),
    ("frame + dek (3D-print)",      0.130, -25.0, 38.0, "PETG, topologisch geoptimaliseerd"),
    ("accu 4$\\times$AA",             0.110, -55.0, 54.0, "hoofddek, achter"),
    ("ATmega328P Xplained Mini",    0.025,  12.0, 54.0, "hoofddek, voor"),
    ("L298N-module",                0.027,  30.0, 80.0, "bovendek"),
    ("breadboard + bedrading",      0.060, -30.0, 71.0, "bovendek"),
    ("draadwerk en bevestiging",    0.030, -20.0, 55.0, ""),
    ("HC-SR04",                     0.010,  45.0, 58.0, "sensorbeugel"),
    ("lijnsensor",                  0.012,  45.0,  8.0, "onder de sensorbeugel"),
    ("vrijloopwieltje",             0.015, -75.0, 14.0, ""),
]
m_tot = sum(m for _, m, _, _, _ in MASS)
x_g = -sum(m * x for _, m, x, _, _ in MASS) / m_tot     # positief = achter de aandrijfas
h_cg = sum(m * z for _, m, _, z, _ in MASS) / m_tot
W = m_tot * g_acc

rec("mtot", m_tot, "{:.3f}")
rec("mtotg", m_tot * 1000, "{:.0f}")
rec("Wn", W, "{:.2f}")
rec("xg", x_g)
rec("hcg", h_cg)

with open(os.path.join(HERE, "massastaat.tex"), "w", encoding="utf-8") as f:
    # de hele tabular staat hier, zodat \input niet midden in een tabelrij eindigt
    f.write("\\begin{tabular}{lrrrl}\n\\toprule\n")
    f.write("Component & $m$ [g] & $x_g$ achter as & $h$ [mm] & Plaats \\\\\n\\midrule\n")
    for name, m, x, z, note in MASS:
        f.write("%s & %.0f & %.0f & %.0f & %s \\\\\n" % (name, m * 1000, (-x) or 0.0, z, note))
    f.write("\\midrule\n\\textbf{totaal} & \\textbf{%.0f} & \\textbf{%.1f} & \\textbf{%.1f} & \\\\\n"
            % (m_tot * 1000, x_g, h_cg))
    f.write("\\bottomrule\n\\end{tabular}\n")

# ==========================================================================
# 4. Statica: lastverdeling, kantelen
# ==========================================================================
def sigma(d):
    """Aandeel van het gewicht op de twee aangedreven wielen."""
    return 1.0 - x_g / d


sig = sigma(d_opt)
sig_lo, sig_hi = 0.65, 0.80
rec("sigmaOpt", 100 * sig)
rec("sigLo", 100 * sig_lo, "{:.0f}")
rec("sigHi", 100 * sig_hi, "{:.0f}")
rec("Ndrive", sig * W, "{:.2f}")
rec("Ncaster", (1 - sig) * W, "{:.2f}")
rec("sensOpt", 100.0 / d_opt, "{:.2f}")
d_sens = 100.0 / 1.5
rec("dSens", d_sens, "{:.0f}")
d_lo_sigma = x_g / (1 - sig_lo)
d_hi_sigma = x_g / (1 - sig_hi)
rec("dLoSigma", d_lo_sigma)
rec("dHiSigma", d_hi_sigma)

# voorwaarts kantelen bij hard remmen -- onafhankelijk van d
a_lift = g_acc * x_g / h_cg
rec("aLift", a_lift, "{:.2f}")
rec("aLiftG", a_lift / g_acc, "{:.2f}")
rec("xgMin", 0.4 * h_cg)


def a_max_accel(d):
    return g_acc * mu * (1 - x_g / d) / (1 + mu * h_cg / d)


rec("aAccel", a_max_accel(d_opt), "{:.2f}")
rec("aAccelG", a_max_accel(d_opt) / g_acc, "{:.2f}")

# zijdelings: glijden voor kantelen
b_roll = 2 * mu * h_cg
rec("bRoll", b_roll)
rec("aRoll", g_acc * (b_opt / 2) / h_cg, "{:.2f}")
rec("aSlip", g_acc * mu, "{:.2f}")


def diag_margin(b, d):
    return (b * (d - x_g) / 2) / np.hypot(d, b / 2)


rec("diagOpt", diag_margin(b_opt, d_opt))
rec("aDiag", g_acc * diag_margin(b_opt, d_opt) / h_cg, "{:.2f}")

# stoorkoppel van het vrijloopwieltje -- valt onafhankelijk van d uit
T_caster = mu_c * W * x_g / 1000.0
rec("Tcaster", T_caster * 1000, "{:.1f}")

# ==========================================================================
# 5. Odometrie
# ==========================================================================
def dtheta_quant(b, N):
    return np.degrees((np.pi * D_wheel / N) / b)


def dtheta_cal(b, turn=90.0):
    return turn * eps_b / b


def drift(b, L=1000.0):
    return L ** 2 * Ed / (2 * b)


rec("dsAxle", np.pi * D_wheel / N_axle, "{:.2f}")
rec("dsShaft", np.pi * D_wheel / N_shaft, "{:.3f}")
rec("quantOpt", dtheta_quant(b_opt, N_axle), "{:.2f}")
rec("quantMin", dtheta_quant(b_min_pack, N_axle), "{:.2f}")
rec("quantShaft", dtheta_quant(b_opt, N_shaft), "{:.3f}")
rec("calOpt", dtheta_cal(b_opt), "{:.2f}")
rec("calMin", dtheta_cal(b_min_pack), "{:.2f}")
rec("driftOpt", drift(b_opt), "{:.0f}")
rec("driftMin", drift(b_min_pack), "{:.0f}")
rec("quantGain", 100 * (1 - dtheta_quant(b_opt, N_axle) / dtheta_quant(b_min_pack, N_axle)), "{:.0f}")

# ==========================================================================
# 6. Aandrijving: 2 versus 4 motoren
# ==========================================================================
F_stall1 = tau_stall / (r_w / 1000.0)
F_trac2 = mu * sig * W
F_trac4 = mu * W
rec("Fstall1", F_stall1, "{:.2f}")
rec("Fstall2", 2 * F_stall1, "{:.2f}")
rec("Ftrac2", F_trac2, "{:.2f}")
rec("Ftrac4", F_trac4, "{:.2f}")
rec("tracGain", 100 * (F_trac4 / F_trac2 - 1), "{:.0f}")


def F_req(a, grade_deg):
    th = np.radians(grade_deg)
    return m_tot * a + C_rr * W * np.cos(th) + W * np.sin(th)


rec("Freq0", F_req(0.5, 0), "{:.2f}")
rec("FreqTien", F_req(0.5, 10), "{:.2f}")
rec("margin0", F_trac2 / F_req(0.5, 0), "{:.1f}")
rec("marginTien", F_trac2 / F_req(0.5, 10), "{:.1f}")
grades = np.linspace(0, 40, 800)
rec("gradeMax2", float(np.interp(F_trac2, F_req(0.5, grades), grades)), "{:.0f}")
rec("gradeMax4", float(np.interp(F_trac4, F_req(0.5, grades), grades)), "{:.0f}")

v_max = np.pi * D_wheel / 1000.0 * n0_rpm / 60.0
rec("vmax", v_max, "{:.2f}")
rec("vwork", v_max * (4.0 / 6.0) * 0.85, "{:.2f}")

T_yaw2 = F_trac2 * (b_opt / 1000.0) / 2
T_res2 = T_caster
L_wb4 = 100.0
T_yaw4 = F_trac4 * (b_opt / 1000.0) / 2
T_res4 = mu * W * (L_wb4 / 1000.0) / 4
rec("Tyaw2", T_yaw2 * 1000, "{:.0f}")
rec("Tres2", T_res2 * 1000, "{:.1f}")
rec("Tyaw4", T_yaw4 * 1000, "{:.0f}")
rec("Tres4", T_res4 * 1000, "{:.0f}")
rec("scrub2", 100 * T_res2 / T_yaw2, "{:.0f}")
rec("scrub4", 100 * T_res4 / T_yaw4, "{:.0f}")
rec("Lwb", L_wb4, "{:.0f}")
rec("Istall2", 2 * I_stall, "{:.1f}")
rec("Istall4", 4 * I_stall, "{:.1f}")
rec("Ichan4", 2 * I_stall, "{:.1f}")
rec("Pdiss4", 2 * I_stall * 2.7, "{:.1f}")

m_end = 0.120
m_core = m_tot - m_end
I0 = m_core * (0.130 ** 2 + 0.070 ** 2) / 12
k_I = 2 * (m_end / 2) * 0.25
rec("bAgile", np.sqrt(I0 / k_I) * 1000, "{:.0f}")


def alpha_yaw(b_mm):
    b = b_mm / 1000.0
    return (F_trac2 * b / 2) / (I0 + k_I * b ** 2)


rec("alphaOpt", alpha_yaw(b_opt), "{:.0f}")
rec("alphaMin", alpha_yaw(b_min_pack), "{:.0f}")
rec("alphaGain", 100 * (alpha_yaw(b_opt) / alpha_yaw(b_min_pack) - 1), "{:.0f}")

# ==========================================================================
# 7. Draaicirkel
# ==========================================================================
def R_swept(b, d, taper=True):
    R_wheel = b / 2 + w_tyre / 2
    R_front = np.hypot(x_front_sens, frame_width(b) / 2)
    R_rear = (d + r_caster_env) if taper else np.hypot(d + r_caster_env, frame_width(b) / 2)
    return np.maximum(R_wheel, np.maximum(R_front, R_rear))


rec("Rswept", R_swept(b_opt, d_opt))
rec("Dswept", 2 * R_swept(b_opt, d_opt))
rec("DsweptSquare", 2 * R_swept(b_opt, d_opt, taper=False))
rec("dMaxFoot", 100.0 - r_caster_env, "{:.0f}")

# ==========================================================================
# 7b. Componentenindeling: welke breedte laat alles op twee dekken passen?
# ==========================================================================
from itertools import permutations, product

clear = 4.0            # speling rondom elk onderdeel
accu_clear = 2.0       # speling naast de accu in het kanaal tussen de motoren
x_rear = d_opt + r_caster_env      # achterkant van het frame, gezet door het wieltje

ITEMS = [("Xplained Mini", PCB["xplained"]), ("breadboard", PCB["breadboard"]),
         ("L298N", PCB["l298"]), ("accu", PCB["accu"])]


def pack_decks(items, W, L):
    """Plankjes-indeling: hoeveel dekken van W (breed) x L (lang) zijn nodig?
    Elk onderdeel mag 90 graden gedraaid worden. 99 = past nooit."""
    best = 99
    for order in permutations(range(len(items))):
        for rots in product((0, 1), repeat=len(items)):
            decks, shelf_w, shelf_l, used_l, ok = 1, 0.0, 0.0, 0.0, True
            for i, rot in zip(order, rots):
                wy, lx = items[i][1]
                if rot:
                    wy, lx = lx, wy
                wy += clear
                lx += clear
                if wy > W or lx > L:
                    ok = False
                    break
                if shelf_w + wy <= W:                 # past naast wat er al staat
                    shelf_w += wy
                    shelf_l = max(shelf_l, lx)
                elif used_l + shelf_l + lx <= L:      # nieuwe rij erachter
                    used_l += shelf_l
                    shelf_w, shelf_l = wy, lx
                else:                                  # nieuw dek
                    decks += 1
                    used_l = 0.0
                    shelf_w, shelf_l = wy, lx
            if ok and used_l + shelf_l <= L:
                best = min(best, decks)
    return best


def layout_for(b, max_decks=2):
    """Geeft (accu in het kanaal?, benodigde x_f, framelengte, frame-oppervlak)."""
    W_u = frame_width_wheel(b) - 6.0                  # bruikbare dekbreedte naast de banden
    in_channel = gap_from_b(b) >= PCB["accu"][0] + accu_clear
    items = [it for it in ITEMS if not (in_channel and it[0] == "accu")]
    for xf in np.arange(x_mot_front, 90.1, 1.0):      # dek loopt minstens tot de motorneus
        if pack_decks(items, W_u, xf + x_rear) <= max_decks:
            L = xf + x_rear
            return in_channel, float(xf), L, L * frame_width(b) / 100.0
    return in_channel, np.nan, np.nan, np.nan


b_scan = np.arange(96.0, 165.0, 2.0)
scan = [layout_for(b) for b in b_scan]
scan_ch = np.array([s[0] for s in scan])
scan_xf = np.array([s[1] for s in scan])
scan_L = np.array([s[2] for s in scan])
scan_A = np.array([s[3] for s in scan])
scan_R = np.array([max(b / 2 + w_tyre / 2, np.hypot(xf, frame_width(b) / 2), x_rear)
                   for b, xf in zip(b_scan, scan_xf)])

b_accu = 2 * (t_gb + e_wheel) + PCB["accu"][0] + accu_clear   # accu past in het kanaal
i_best = int(np.nanargmin(scan_A))
b_area = float(b_scan[i_best])
b_swept_max = float(b_scan[scan_R <= x_rear + 0.01][-1])

rec("bAccu", b_accu)
rec("bArea", b_area, "{:.0f}")
rec("Aframe", scan_A[i_best])
rec("AframeSmal", float(np.nanmax(scan_A[b_scan < b_accu])))
rec("bSmalMin", float(b_scan[~np.isnan(scan_A)][0]), "{:.0f}")
rec("AframeBreed", float(np.interp(152.0, b_scan, scan_A)))
rec("AframeGain", 100 * (float(np.interp(152.0, b_scan, scan_A)) / scan_A[i_best] - 1), "{:.0f}")
rec("bSweptMax", b_swept_max, "{:.0f}")
rec("xfOpt", float(np.interp(b_opt, b_scan, scan_xf)), "{:.0f}")
rec("xfSmal", float(scan_xf[0]), "{:.0f}")
# ==========================================================================
# 7c. Maximaal toelaatbaar gewicht van de hele auto
# ==========================================================================
# Lineair gelijkstroommotormodel bij vaste klemspanning U:
#   tau(u) = u * tau_s   met u = tau/tau_s
#   omega(u) = omega_0 (1 - u)
#   I(u) = I_0 + (I_s - I_0) u
# Rendement eta = tau*omega/(U*I). d(eta)/du = 0 geeft een kwadratische
# vergelijking met als wortel het werkpunt van het beste rendement:
#   u* = (sqrt(I_0 I_s) - I_0) / (I_s - I_0)
# Dat werkpunt wordt aangehouden als duurzame belasting: het levert het meeste
# nuttige werk per Ah accu en daarmee ook de minste warmte in de motor.
rec("Ifree", I_free, "{:.2f}")
rec("Upack", U_pack, "{:.1f}")
rec("Um", U_m, "{:.2f}")
rec("Vdrop", V_d0 + V_d1 * I_cont, "{:.2f}")
rec("Rpack", 4 * R_cell, "{:.2f}")
rec("tauStallZes", tau_stall6 * 1000, "{:.0f}")
rec("IstallZes", I_stall6, "{:.2f}")
rec("nNulZes", n0_rpm, "{:.0f}")
rec("Ubat", U_ref, "{:.0f}")
rec("omegaNul", omega0, "{:.1f}")
rec("tauCont", tau_cont * 1000, "{:.1f}")
rec("omegaCont", omega_cont, "{:.1f}")
rec("nCont", omega_cont * 60 / (2 * np.pi), "{:.0f}")
rec("vCont", omega_cont * r_w / 1000.0, "{:.2f}")
rec("IcontEen", I_cont, "{:.2f}")
rec("etaCont", 100 * eta_cont, "{:.0f}")
rec("PmechCont", 2 * tau_cont * omega_cont, "{:.2f}")
rec("PelekCont", 2 * U_m * I_cont, "{:.2f}")


def m_max(a, grade_deg):
    """Zwaarste auto die de motoren nog kunnen versnellen op deze helling [kg]."""
    th = np.radians(grade_deg)
    return F_cont / (a + g_acc * (C_rr * np.cos(th) + np.sin(th)))


# alternatief: dezelfde cellen, maar een MOSFET-driver in plaats van de L298
def keten(vd0, vd1):
    U = U_pack
    for _ in range(200):
        sv = U / U_ref
        ts, w0, Is = tau_stall6 * sv, omega0_ref * sv, I_stall6 * sv
        k = (np.sqrt(I_free * Is) - I_free) / (Is - I_free)
        I1 = I_free + (Is - I_free) * k
        U += 0.4 * ((U_pack - 2 * I1 * R_pack - (vd0 + vd1 * I1)) - U)
    return U, 2 * k * ts / (r_w / 1000.0), w0 * (1 - k) * r_w / 1000.0


U_mos, F_mos, v_mos = keten(0.5, 0.3)
rec("UmMos", U_mos, "{:.2f}")
rec("vMos", v_mos, "{:.2f}")

m_comp = m_tot - 0.130          # alles behalve het frame
m_advies = m_max(0.3, 5.0)      # maatgevend geval: drempel van 5 graden

rec("kDuty", 100 * k_duty, "{:.1f}")
rec("Fcont", F_cont, "{:.2f}")
rec("mMaxVlak", 1000 * m_max(0.5, 0.0), "{:.0f}")
rec("mMaxSnel", 1000 * m_max(1.0, 0.0), "{:.0f}")
rec("mMaxVijf", 1000 * m_max(0.3, 5.0), "{:.0f}")
rec("mMaxTien", 1000 * m_max(0.3, 10.0), "{:.0f}")
rec("mAdvies", 1000 * m_advies, "{:.0f}")
rec("mComp", 1000 * m_comp, "{:.0f}")
rec("mFrameNu", 130, "{:.0f}")
rec("mRuimte", 1000 * (m_advies - m_tot), "{:.0f}")
rec("mFrameMax", 1000 * (m_advies - m_comp), "{:.0f}")
rec("Icont", k_duty * I_stall + 0.10, "{:.2f}")
rec("aNu", F_cont / m_tot - g_acc * C_rr, "{:.2f}")
rec("mMosVijf", 1000 * F_mos / (0.3 + g_acc * (C_rr * np.cos(np.radians(5)) + np.sin(np.radians(5)))), "{:.0f}")
rec("mMosWinst", 100 * (F_mos / F_cont - 1), "{:.0f}")

# ==========================================================================
# 7d. Bouwhoogte: de verticale stapeling
# ==========================================================================
h_deck_plate = 3.0        # dikte van een dekplaat
h_xplained = 15.0         # Xplained Mini inclusief headers
h_l298 = 27.0             # L298N-module inclusief koellichaam
h_breadboard = 10.0
h_accu = 15.0             # platte 4xAA houder
h_clear = 2.0             # montagespeling tussen de dekken

z_axle = r_w                                   # aandrijfas
z_mot_lo, z_mot_hi = z_axle - h_mot / 2, z_axle + h_mot / 2
z_deck1 = z_mot_hi + h_deck_plate              # bovenkant hoofddek
z_ground = z_mot_lo                            # motorhuis is het laagste punt
z_accu_lo = z_deck1                            # platte houder staat op het hoofddek
z_tall = z_deck1 + max(h_accu, h_xplained)     # hoogste onderdeel op het hoofddek
z_deck2 = z_tall + h_clear + h_deck_plate      # bovenkant bovendek
z_top = z_deck2 + h_l298                       # L298N met koellichaam is het hoogst
h_standoff = z_deck2 - h_deck_plate - z_deck1  # benodigde afstandbussen

# bovengrenzen aan het zwaartepunt
h_max_pitch = x_g / 0.40                       # a_kantel >= 0,4 g
h_max_roll = b_opt / (2 * mu)                  # glijdt voor het kantelt
h_max = min(h_max_pitch, h_max_roll)
Sum_mz = sum(m * z for _, m, _, z, _ in MASS)
m_top_max = (h_max * m_tot - Sum_mz) / (z_top - 5.0 - h_max)   # extra massa op het bovendek

beta = np.radians(7.5)                          # halve openingshoek HC-SR04
z_sr04 = 58.0
rec("zAxle", z_axle, "{:.0f}")
rec("zMotLo", z_mot_lo, "{:.0f}")
rec("zMotHi", z_mot_hi, "{:.0f}")
rec("zDeckEen", z_deck1, "{:.0f}")
rec("zDeckTwee", z_deck2, "{:.0f}")
rec("zAccuLo", z_accu_lo, "{:.0f}")
rec("zTop", z_top, "{:.0f}")
rec("hStandoff", h_standoff, "{:.0f}")
rec("hLTweeNegenAcht", h_l298, "{:.0f}")
rec("zTyre", D_wheel, "{:.0f}")
rec("hMaxPitch", h_max_pitch)
rec("hMaxRoll", h_max_roll)
rec("hMax", h_max)
rec("mTopMax", 1000 * m_top_max, "{:.0f}")
rec("hMarge", h_max - h_cg)
rec("Rfloor", z_sr04 / np.tan(beta), "{:.0f}")
rec("zSRvier", z_sr04, "{:.0f}")
rec("zLijn", 8.0, "{:.0f}")
rec("clearGround", z_ground, "{:.0f}")
rec("breakover", 2 * np.degrees(np.arctan(z_ground / (d_opt / 2))), "{:.0f}")

rec("xmotf", x_mot_front, "{:.0f}")
rec("accuW", PCB["accu"][0], "{:.0f}")
rec("accuL", PCB["accu"][1], "{:.0f}")
rec("clear", clear, "{:.0f}")
rec("xrear", x_rear, "{:.0f}")
rec("quantBreed", dtheta_quant(152.0, N_axle), "{:.2f}")
rec("quantWinst", 100 * (1 - dtheta_quant(152.0, N_axle) / dtheta_quant(b_opt, N_axle)), "{:.0f}")

for k, v in [("bOpt", b_opt), ("dOpt", d_opt), ("rw", r_w), ("Dw", D_wheel),
             ("wtyre", w_tyre), ("tgb", t_gb), ("ewheel", e_wheel), ("Lmot", L_mot),
             ("hmot", h_mot), ("mu", mu), ("muc", mu_c),
             ("Lframe", x_front_sens + d_opt + r_caster_env),
             ("epsb", eps_b), ("xmotr", x_mot_rear), ("crr", C_rr)]:
    rec(k, v, "{:.2f}" if v < 1 else ("{:.1f}" if v % 1 else "{:.0f}"))
rec("Ed", 100 * Ed, "{:.0f}")
rec("Naxle", N_axle, "{:.0f}")
rec("Nshaft", N_shaft, "{:.0f}")
rec("taustall", tau_stall * 1000, "{:.0f}")
rec("Istall1", I_stall, "{:.1f}")
rec("holeX", cast_hole_x, "{:.0f}")
rec("holeY", cast_hole_y, "{:.0f}")

# ==========================================================================
#  Figuur 1 -- definities
# ==========================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.9))
b, d = b_opt, d_opt
ax1.add_patch(Polygon([[0, b / 2], [0, -b / 2], [-d, 0]], closed=True,
                      fc=C["light"], ec=C["grey"], lw=0.8, ls="--", alpha=0.7, zorder=0))
for s in (+1, -1):
    ax1.add_patch(Rectangle((-r_w, s * b / 2 - w_tyre / 2), 2 * r_w, w_tyre,
                            fc=C["blue"], alpha=0.35, ec=C["blue"], lw=1.0))
ax1.add_patch(Circle((-d, 0), 9, fc=C["orange"], alpha=0.5, ec=C["orange"]))
ax1.plot([0, 0], [-b / 2, b / 2], color=C["grey"], lw=1.0)
ax1.plot(-x_g, 0, marker="o", ms=6, color=C["red"], zorder=5)
ax1.annotate("", xy=(6, -b / 2), xytext=(6, b / 2), arrowprops=dict(arrowstyle="<->", color=C["blue"], lw=1.1))
ax1.text(10, 0, "$b$", color=C["blue"], va="center", fontsize=10)
ax1.annotate("", xy=(-d, -36), xytext=(0, -36), arrowprops=dict(arrowstyle="<->", color=C["orange"], lw=1.1))
ax1.text(-d / 2, -42, "$d$", color=C["orange"], ha="center", va="top", fontsize=10)
ax1.annotate("", xy=(-x_g, 12), xytext=(0, 12), arrowprops=dict(arrowstyle="<->", color=C["red"], lw=1.0))
ax1.text(-x_g / 2, 15, "$x_g$", color=C["red"], ha="center", fontsize=9)
ax1.text(-d, 14, "vrijloopwiel", color=C["orange"], ha="center", fontsize=7)
ax1.text(-112, 68, "rijrichting $\\rightarrow$", fontsize=7, color=C["grey"])
ax1.set_xlim(-115, 55); ax1.set_ylim(-82, 82); ax1.set_aspect("equal")
ax1.set_title("bovenaanzicht: steundriehoek")
ax1.set_xlabel("x [mm]"); ax1.set_ylabel("y [mm]")

ax2.axhline(0, color=C["grey"], lw=1.2)
ax2.add_patch(Circle((0, r_w), r_w, fc="none", ec=C["blue"], lw=1.4))
ax2.add_patch(Circle((-d, 9), 9, fc="none", ec=C["orange"], lw=1.4))
ax2.add_patch(Rectangle((-d - 15, 45), d + 15 + x_front_sens, 4, fc=C["light"], ec=C["grey"], lw=0.8))
ax2.plot(-x_g, h_cg, marker="o", ms=6, color=C["red"])
ax2.text(-x_g + 5, h_cg + 4, "zwaartepunt", color=C["red"], ha="left", fontsize=7)
ax2.annotate("", xy=(-x_g, 0), xytext=(-x_g, h_cg), arrowprops=dict(arrowstyle="<->", color=C["red"], lw=1.0))
ax2.text(-x_g - 4, h_cg / 2, "$h$", color=C["red"], ha="right", fontsize=10)
ax2.annotate("", xy=(40, 0), xytext=(40, r_w), arrowprops=dict(arrowstyle="<->", color=C["blue"], lw=1.0))
ax2.text(43, r_w / 2, "$r$", color=C["blue"], fontsize=10)
ax2.set_xlim(-115, 55); ax2.set_ylim(-8, 78); ax2.set_aspect("equal")
ax2.set_title("zijaanzicht")
ax2.set_xlabel("x [mm]"); ax2.set_ylabel("z [mm]")
save(fig, "f1_definities")

# ==========================================================================
#  Figuur 2 -- lastverdeling, gevoeligheid, kantelen
# ==========================================================================
fig, axs = plt.subplots(1, 3, figsize=(7.4, 2.7), layout="constrained")
dd = np.linspace(35, 130, 400)

ax = axs[0]
ax.plot(dd, 100 * sigma(dd), color=C["blue"], lw=1.7, label="aandrijfwielen $\\sigma$")
ax.plot(dd, 100 * (1 - sigma(dd)), color=C["orange"], lw=1.7, label="vrijloopwiel")
ax.axhspan(100 * sig_lo, 100 * sig_hi, color=C["green"], alpha=0.13)
ax.text(37, 100 * (sig_lo + sig_hi) / 2, "werkgebied", fontsize=6.5, color=C["green"], va="center")
ax.axvline(d_opt, color=C["red"], ls="--", lw=1.0)
ax.plot(d_opt, 100 * sig, "o", ms=5, color=C["red"])
ax.set_xlabel("naloopafstand $d$ [mm]"); ax.set_ylabel("aandeel van het gewicht [%]")
ax.set_title(f"lastverdeling bij $x_g$ = {x_g:.0f} mm")
ax.legend(loc="center right"); ax.set_xlim(35, 130); ax.set_ylim(0, 100)

ax = axs[1]
ax.plot(dd, 100 / dd, color=C["blue"], lw=1.7)
ax.axhline(1.5, color=C["grey"], ls=":", lw=1.0)
ax.text(37, 1.6, "eis $\\leq$ 1,5 %-punt/mm", fontsize=6.5, color=C["grey"])
ax.axvline(d_sens, color=C["green"], ls="--", lw=1.0)
ax.text(d_sens + 3, 2.8, f"$d\\geq$ {d_sens:.0f} mm", color=C["green"], fontsize=6.5)
ax.plot(d_opt, 100 / d_opt, "o", ms=5, color=C["red"])
ax.set_xlabel("naloopafstand $d$ [mm]")
ax.set_ylabel("$|\\partial\\sigma/\\partial x_g|$ [%-punt/mm]")
ax.set_title("gevoeligheid voor bouwtoleranties")
ax.set_xlim(35, 130); ax.set_ylim(0, 3.2)

ax = axs[2]
hh = np.linspace(25, 70, 300)
for xg_v, col, lw in ((12.0, C["light"], 1.4), (x_g, C["blue"], 1.8), (30.0, C["teal"], 1.4)):
    ax.plot(hh, xg_v / hh, color=col, lw=lw, label=f"$x_g$ = {xg_v:.0f} mm")
ax.axhline(0.4, color=C["grey"], ls=":", lw=1.0)
ax.text(26, 0.43, "eis 0,4 $g$", fontsize=6.5, color=C["grey"])
ax.plot(h_cg, a_lift / g_acc, "o", ms=5, color=C["red"])
ax.set_xlabel("hoogte zwaartepunt $h$ [mm]")
ax.set_ylabel("remvertraging bij optillen [$g$]")
ax.set_title("voorwaarts kantelen")
ax.legend(loc="upper right"); ax.set_xlim(25, 70); ax.set_ylim(0, 1.3)
save(fig, "f2_lastverdeling")

# ==========================================================================
#  Figuur 3 -- odometrie
# ==========================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.7))
bb = np.linspace(70, 150, 400)
ax1.plot(bb, dtheta_quant(bb, N_axle), color=C["blue"], lw=1.7,
         label=f"quantisatie, schijf op wielas ({N_axle} p/omw)")
ax1.plot(bb, dtheta_cal(bb), color=C["orange"], lw=1.7,
         label="fout na 90$^\\circ$ door $\\epsilon_b$ = 1,5 mm")
ax1.plot(bb, dtheta_quant(bb, N_shaft) * 10, color=C["green"], lw=1.4, ls="--",
         label="quantisatie, schijf op motoras ($\\times$10)")
ax1.axvline(b_opt, color=C["red"], lw=1.0, ls="--")
ax1.axvspan(70, b_min_pack, color=C["grey"], alpha=0.18)
ax1.text(71.5, 0.4, "motoren\nraken elkaar", fontsize=6.0, color=C["grey"])
ax1.set_xlabel("spoorbreedte $b$ [mm]"); ax1.set_ylabel("hoekfout [$^\\circ$]")
ax1.set_title("hoekfout van de odometrie")
ax1.legend(loc="upper right"); ax1.set_xlim(70, 150); ax1.set_ylim(0, 8.6)

ax2.plot(bb, drift(bb), color=C["purple"], lw=1.7)
ax2.axvline(b_opt, color=C["red"], lw=1.0, ls="--")
ax2.plot(b_opt, drift(b_opt), "o", ms=5, color=C["red"])
ax2.annotate(f"{drift(b_opt):.0f} mm bij $b$ = {b_opt:.0f} mm", xy=(b_opt, drift(b_opt)),
             xytext=(b_opt + 5, drift(b_opt) + 16), fontsize=7, color=C["red"],
             arrowprops=dict(arrowstyle="-", color=C["red"], lw=0.8))
ax2.axvspan(70, b_min_pack, color=C["grey"], alpha=0.18)
ax2.set_xlabel("spoorbreedte $b$ [mm]"); ax2.set_ylabel("zijdelingse afwijking [mm]")
ax2.set_title(f"koersdrift over 1 m bij {100*Ed:.0f} % wielverschil")
ax2.set_xlim(70, 150)
save(fig, "f3_odometrie")

# ==========================================================================
#  Figuur 4 -- ontwerpruimte
# ==========================================================================
fig, ax = plt.subplots(figsize=(6.4, 4.3))
bg = np.linspace(70, 165, 500)
dg = np.linspace(35, 130, 500)
B, Dg = np.meshgrid(bg, dg)

feas = ((B >= b_accu) & (B <= b_swept_max) & (B >= b_roll)
        & (Dg >= d_lo_sigma) & (Dg <= d_hi_sigma)
        & (Dg >= d_sens) & (2 * R_swept(B, Dg) <= 200.0))
ax.contourf(B, Dg, feas.astype(float), levels=[0.5, 1.5], colors=[C["green"]], alpha=0.20)
lv = [150, 170, 190, 210, 230, 250]
cs = ax.contour(B, Dg, 2 * R_swept(B, Dg), levels=lv,
                colors=C["grey"], linewidths=0.7, linestyles=":")
ax.clabel(cs, fmt="$\\varnothing$%.0f", fontsize=6.5,
          manual=[(96, v / 2 - r_caster_env) for v in lv])

ax.axvline(b_accu, color=C["blue"], lw=1.3)
ax.text(b_accu + 1.5, 128, "accu past niet tussen de motoren", rotation=90, va="top",
        fontsize=6.5, color=C["blue"])
ax.axvline(b_swept_max, color=C["blue"], lw=1.3)
ax.text(b_swept_max - 2, 128, "wielen bepalen de draaicirkel", rotation=90, va="top", ha="right",
        fontsize=6.5, color=C["blue"])
ax.axhline(d_sens, color=C["orange"], lw=1.3)
ax.text(72, d_sens - 4.5, "gevoeligheid $>$ 1,5 %-punt/mm", ha="left", fontsize=6.5, color=C["orange"])
ax.axhline(d_hi_sigma, color=C["teal"], lw=1.3, ls="--")
ax.text(148, d_hi_sigma + 1.5, f"$\\sigma > $ {100*sig_hi:.0f} %: vrijloopwiel te licht",
        ha="right", fontsize=6.5, color=C["teal"])
ax.axhline(100.0 - r_caster_env, color=C["red"], lw=1.3, ls=":")
ax.text(72, 100 - r_caster_env + 1.5, "draaicirkel $>$ 200 mm", fontsize=6.5, color=C["red"])

ax.plot(b_opt, d_opt, marker="*", ms=17, color=C["red"], mec="white", mew=0.8, zorder=6)
ax.annotate(f"keuze\n$b$ = {b_opt:.0f} mm\n$d$ = {d_opt:.0f} mm", xy=(b_opt, d_opt),
            xytext=(b_opt + 11, d_opt - 22), fontsize=7.5, color=C["red"],
            arrowprops=dict(arrowstyle="->", color=C["red"], lw=0.9))
ax.set_xlabel("spoorbreedte $b$ [mm]"); ax.set_ylabel("naloopafstand $d$ [mm]")
ax.set_title("ontwerpruimte -- groen voldoet aan alle eisen;\ngestippeld: diameter van de draaicirkel [mm]")
ax.set_xlim(70, 165); ax.set_ylim(35, 130)
save(fig, "f4_ontwerpruimte")

# ==========================================================================
#  Figuur 5 -- aandrijfbudget
# ==========================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.9))
gr = np.linspace(0, 32, 500)
ax1.fill_between(gr, 0, F_req(0.5, gr), color=C["light"], alpha=0.8)
ax1.plot(gr, F_req(0.5, gr), color=C["grey"], lw=1.7, label="benodigd, $a$ = 0,5 m/s$^2$")
ax1.axhline(F_trac2, color=C["blue"], lw=1.6, label="grip, 2 aangedreven wielen")
ax1.axhline(F_trac4, color=C["green"], lw=1.4, ls="--", label="grip, 4 aangedreven wielen")
ax1.axhline(2 * F_stall1, color=C["orange"], lw=1.4, ls=":", label="blokkeerkracht 2 motoren")
gi = float(np.interp(F_trac2, F_req(0.5, gr), gr))
ax1.plot(gi, F_trac2, "o", ms=5, color=C["red"])
ax1.text(gi - 0.8, F_trac2 + 0.18, f"2WD haalt {gi:.0f}$^\\circ$", ha="right", fontsize=7, color=C["red"])
ax1.set_xlabel("hellingshoek [$^\\circ$]"); ax1.set_ylabel("trekkracht [N]")
ax1.set_title("trekkrachtbudget"); ax1.legend(loc="upper left")
ax1.set_xlim(0, 32); ax1.set_ylim(0, 5.6)

lbl = ["giermoment\nbeschikbaar\n[mNm]", "verlies aan\nschuifwrijving\n[mNm]", "blokkeerstroom\nper L298-kanaal\n[100 mA]"]
v2 = [T_yaw2 * 1000, T_res2 * 1000, I_stall * 10]
v4 = [T_yaw4 * 1000, T_res4 * 1000, 2 * I_stall * 10]
x = np.arange(3); wb = 0.36
ax2.bar(x - wb / 2, v2, wb, color=C["blue"], label="2 motoren + vrijloopwiel")
ax2.bar(x + wb / 2, v4, wb, color=C["orange"], label="4 motoren (skid steer)")
for xi, (a_, b_) in enumerate(zip(v2, v4)):
    ax2.text(xi - wb / 2, a_ + 4, f"{a_:.0f}", ha="center", fontsize=6.5)
    ax2.text(xi + wb / 2, b_ + 4, f"{b_:.0f}", ha="center", fontsize=6.5)
ax2.set_xticks(x); ax2.set_xticklabels(lbl, fontsize=6.5)
ax2.set_title("draaien op de plaats, en belasting van de L298")
ax2.legend(loc="upper center"); ax2.set_ylim(0, 300)
save(fig, "f5_aandrijving")

# ==========================================================================
#  Figuur 6 -- indeling op schaal
# ==========================================================================
fig, ax = plt.subplots(figsize=(6.6, 5.4))
gap = gap_from_b(b_opt)
fw = frame_width(b_opt)
Rr = d_opt + r_caster_env

fww = frame_width_wheel(b_opt)      # smaller naast de banden
x_notch = -r_w                       # daar begint de bandzone
arc_y = np.linspace(-fw / 2, fw / 2, 160)
arc_x = -np.sqrt(np.maximum(Rr ** 2 - arc_y ** 2, 0))
frame_poly = (list(zip(arc_x, arc_y))
              + [(x_notch, fw / 2), (x_notch, fww / 2), (x_front_deck, fww / 2),
                 (x_front_deck, -fww / 2), (x_notch, -fww / 2), (x_notch, -fw / 2)])
ax.add_patch(Polygon(frame_poly, closed=True, fc="#f4f4f4", ec=C["grey"], lw=1.3, zorder=0))
ax.annotate(f"frame {fww:.0f} mm breed naast de banden\n({(b_opt-w_tyre)/2 - fww/2:.1f} mm speling)",
            xy=(-32, -fww / 2), xytext=(-119, -80), fontsize=6.5, color=C["grey"], ha="left",
            arrowprops=dict(arrowstyle="-", color=C["grey"], lw=0.7))
ax.add_patch(Polygon([(x_front_deck, 24), (x_front_sens, 24), (x_front_sens, -24), (x_front_deck, -24)],
                     closed=True, fc="#f4f4f4", ec=C["grey"], lw=1.0, ls="--", zorder=0))
ax.text(x_front_sens + 2, 0, "sensorbeugel", fontsize=6.5, color=C["grey"], ha="left", va="center", rotation=90)

for s in (+1, -1):
    ax.add_patch(Rectangle((-r_w, s * b_opt / 2 - w_tyre / 2), 2 * r_w, w_tyre,
                           fc=C["blue"], alpha=0.30, ec=C["blue"], lw=1.2, zorder=4))
    y0 = gap / 2 if s > 0 else -gap / 2 - t_gb
    ax.add_patch(Rectangle((-x_mot_rear, y0), L_mot, t_gb, fc="none", ec="#4a4a4a",
                           lw=1.0, hatch="////", zorder=6))
ax.text(-x_mot_rear + 3, gap / 2 + t_gb / 2, "motor", fontsize=6.5, color="#3d3d3d", va="center", zorder=7)
ax.text(-r_w - 4, b_opt / 2, "wiel", fontsize=6.5, color=C["blue"], va="center", ha="right")

ax.add_patch(Circle((-d_opt, 0), r_caster_env, fc=C["orange"], alpha=0.40, ec=C["orange"], lw=1.2, zorder=4))
ax.add_patch(Rectangle((-d_opt - cast_hole_x / 2, -cast_hole_y / 2), cast_hole_x, cast_hole_y,
                       fc="none", ec=C["orange"], lw=0.8, ls=":", zorder=5))
for sx in (-1, 1):
    for sy in (-1, 1):
        ax.plot(-d_opt + sx * cast_hole_x / 2, sy * cast_hole_y / 2, "o", ms=2.4, color=C["orange"], zorder=6)
ax.annotate("vrijloopwiel onder het frame,\ngatenpatroon 30$\\times$25",
            xy=(-d_opt, -r_caster_env), xytext=(-118, -92), fontsize=6.5, color=C["orange"],
            ha="left", va="top", arrowprops=dict(arrowstyle="-", color=C["orange"], lw=0.7))

items = [
    ("1  accu 4$\\times$AA, hoofddek achter", PCB["accu"], -55.0, C["green"], "-", (-84, 22)),
    ("2  Xplained Mini, hoofddek voor", PCB["xplained"], 12.0, C["purple"], "-", (46, 24)),
    ("3  breadboard, bovendek", PCB["breadboard"], -30.0, C["orange"], "--", (-8, 22)),
    ("4  L298N, bovendek", PCB["l298"], 30.0, C["teal"], "--", (30, 15)),
]
for i, (name, (wy, lx), cx, col, ls, lp) in enumerate(items, start=1):
    ax.add_patch(Rectangle((cx - lx / 2, -wy / 2), lx, wy, fc=col, alpha=0.13,
                           ec=col, lw=1.1, ls=ls, zorder=3, label=name))
    ax.text(lp[0], lp[1], str(i), fontsize=9, color=col, ha="center", va="center",
            zorder=9, fontweight="bold")

ax.plot(-x_g, 0, marker="o", ms=8, color=C["red"], zorder=8)
ax.add_patch(Circle((-x_g, 0), 8, fc="none", ec=C["red"], lw=0.9, ls=":", zorder=8))
ax.add_patch(Circle((0, 0), R_swept(b_opt, d_opt), fc="none", ec=C["red"], lw=1.0, ls="-.", zorder=5))
ax.text(8, R_swept(b_opt, d_opt) + 3, f"draaicirkel $\\varnothing$ {2*R_swept(b_opt,d_opt):.0f} mm",
        fontsize=7, color=C["red"], ha="left")

xb = 78
for s in (+1, -1):
    ax.plot([r_w, xb + 3], [s * b_opt / 2, s * b_opt / 2], color=C["blue"], lw=0.6, ls="-")
ax.annotate("", xy=(xb, -b_opt / 2), xytext=(xb, b_opt / 2), arrowprops=dict(arrowstyle="<->", color=C["blue"], lw=1.2))
ax.text(xb - 3, 0, f"$b$ = {b_opt:.0f}", color=C["blue"], va="center", fontsize=8.5, rotation=90, ha="right")
ax.annotate("", xy=(-d_opt, -fw / 2 - 12), xytext=(0, -fw / 2 - 12),
            arrowprops=dict(arrowstyle="<->", color=C["orange"], lw=1.2))
ax.text(-d_opt / 2, -fw / 2 - 15, f"$d$ = {d_opt:.0f}", color=C["orange"], ha="center", va="top",
        fontsize=8.5, bbox=dict(fc="white", ec="none", pad=0.6))
ax.annotate("", xy=(-x_g, b_opt / 2 + 20), xytext=(0, b_opt / 2 + 20),
            arrowprops=dict(arrowstyle="<->", color=C["red"], lw=1.0))
ax.plot([-x_g, -x_g], [0, b_opt / 2 + 20], color=C["red"], lw=0.6, ls=":")
ax.text(-x_g / 2, b_opt / 2 + 22, f"$x_g$ = {x_g:.0f}", color=C["red"], ha="center", va="bottom", fontsize=7.5)
ax.annotate("", xy=(-97, -fw / 2), xytext=(-97, fw / 2), arrowprops=dict(arrowstyle="<->", color=C["grey"], lw=1.0))
ax.text(-100, 0, f"frame {fw:.0f}", color=C["grey"], va="center", ha="right", fontsize=7.5, rotation=90)

ax.legend(loc="lower right", framealpha=0.93, borderpad=0.5)
ax.set_xlim(-120, 95); ax.set_ylim(-108, 108); ax.set_aspect("equal")
ax.set_xlabel("x [mm]   (rijrichting naar rechts)"); ax.set_ylabel("y [mm]")
ax.set_title("aanbevolen indeling, op schaal")
save(fig, "f6_indeling")

# ==========================================================================
#  Figuur 7 -- breedte uit de componenten
# ==========================================================================
fig, axs = plt.subplots(1, 3, figsize=(7.4, 2.7), layout="constrained")

ax = axs[0]
ax.plot(b_scan, scan_L, color=C["blue"], lw=1.8)
ax.axvline(b_accu, color=C["green"], lw=1.2, ls="--")
ax.text(b_accu + 1.5, 196, "accu past in\nhet kanaal", fontsize=6.5, color=C["green"], va="top")
ax.axvline(b_opt, color=C["red"], lw=1.0, ls=":")
ax.set_xlabel("spoorbreedte $b$ [mm]"); ax.set_ylabel("framelengte [mm]")
ax.set_title("lengte die nodig is om alles\nop twee dekken te krijgen", fontsize=8.5)
ax.set_xlim(96, 164); ax.set_ylim(90, 200)

ax = axs[1]
ax.plot(b_scan, scan_A, color=C["purple"], lw=1.8)
ax.plot(b_area, scan_A[i_best], "o", ms=6, color=C["red"])
ax.annotate(f"minimum bij\n$b$ = {b_area:.0f} mm", xy=(b_area, scan_A[i_best]),
            xytext=(b_area + 13, scan_A[i_best] + 15), fontsize=7, color=C["red"],
            arrowprops=dict(arrowstyle="->", color=C["red"], lw=0.9))
ax.set_xlabel("spoorbreedte $b$ [mm]"); ax.set_ylabel("frame-oppervlak [cm$^2$]")
ax.set_title("materiaal en massa\nvan het frame", fontsize=8.5)
ax.set_xlim(96, 164)

ax = axs[2]
ax.plot(b_scan, 2 * scan_R, color=C["orange"], lw=1.8, label="draaicirkel")
ax.plot(b_scan, b_scan + w_tyre, color=C["teal"], lw=1.5, ls="--", label="breedte over de banden")
ax.axvline(b_swept_max, color=C["grey"], lw=1.0, ls=":")
ax.text(b_swept_max - 2, 252, "hier nemen de\nwielen het over", fontsize=6.5,
        color=C["grey"], ha="right", va="top")
ax.plot(b_opt, 2 * np.interp(b_opt, b_scan, scan_R), "o", ms=6, color=C["red"])
ax.set_xlabel("spoorbreedte $b$ [mm]"); ax.set_ylabel("[mm]")
ax.set_title("wat extra breedte kost\naan buitenmaat", fontsize=8.5)
ax.legend(loc="lower right"); ax.set_xlim(96, 164); ax.set_ylim(110, 265)
save(fig, "f7_breedte")

# ==========================================================================
#  Figuur 8 -- maximaal gewicht
# ==========================================================================
fig, (ax0, ax1, ax2) = plt.subplots(1, 3, figsize=(7.4, 2.7), layout="constrained")

uu = np.linspace(0.001, 1.0, 400)
ax0.plot(100 * uu, omega0 * (1 - uu) * 60 / (2 * np.pi), color=C["blue"], lw=1.7,
         label="toerental [rpm]")
ax0.plot(100 * uu, 100 * (I_free + (I_stall - I_free) * uu), color=C["orange"], lw=1.5,
         label="stroom [10 mA]")
eta = uu * tau_stall * omega0 * (1 - uu) / (U_m * (I_free + (I_stall - I_free) * uu))
ax0.plot(100 * uu, 1000 * eta, color=C["green"], lw=1.7, label="rendement [0,1 %]")
ax0.axvline(100 * k_duty, color=C["red"], lw=1.2, ls="--")
ax0.annotate(f"werkpunt\n$u^*$ = {100*k_duty:.0f} %", xy=(100 * k_duty, 60),
             xytext=(100 * k_duty + 12, 55), fontsize=7, color=C["red"],
             arrowprops=dict(arrowstyle="->", color=C["red"], lw=0.9))
ax0.set_xlabel("belasting $u = \\tau/\\tau_s$ [%]")
ax0.set_title("motorkarakteristiek bij $U_m$")
ax0.legend(loc="upper right"); ax0.set_xlim(0, 100); ax0.set_ylim(0, 210)

gg = np.linspace(0, 15, 300)
for a_v, col, lab in ((0.3, C["blue"], "$a$ = 0,3 m/s$^2$"),
                      (0.5, C["teal"], "$a$ = 0,5 m/s$^2$"),
                      (1.0, C["purple"], "$a$ = 1,0 m/s$^2$")):
    ax1.plot(gg, 1000 * m_max(a_v, gg), color=col, lw=1.7, label=lab)
ax1.axhline(1000 * m_tot, color=C["green"], lw=1.2, ls="--")
ax1.text(14.5, 1000 * m_tot + 25, f"ontwerp nu {1000*m_tot:.0f} g", ha="right",
         fontsize=6.5, color=C["green"])
ax1.plot(5.0, 1000 * m_advies, "*", ms=15, color=C["red"], mec="white", mew=0.7, zorder=5)
ax1.annotate(f"advies: {1000*m_advies:.0f} g", xy=(5.0, 1000 * m_advies),
             xytext=(7.0, 1450), fontsize=7.5, color=C["red"],
             arrowprops=dict(arrowstyle="->", color=C["red"], lw=0.9))
ax1.set_xlabel("hellingshoek [$^\\circ$]"); ax1.set_ylabel("maximale totale massa [g]")
ax1.set_title("wat twee motoren duurzaam trekken")
ax1.legend(loc="lower left"); ax1.set_xlim(0, 15); ax1.set_ylim(0, 2000)

parts = [("componenten", 1000 * m_comp, C["blue"]),
         ("frame nu", 115.0, C["teal"]),
         ("nog vrij", 1000 * (m_advies - m_tot), C["light"])]
left = 0.0
for lab, val, col in parts:
    ax2.barh(0, val, left=left, height=0.45, color=col, edgecolor="white", lw=1.2)
    ax2.text(left + val / 2, 0, f"{lab}\n{val:.0f} g", ha="center", va="center", fontsize=7,
             color="#222222")
    left += val
ax2.axvline(1000 * m_advies, color=C["red"], lw=1.5)
ax2.text(1000 * m_advies - 10, 0.34, f"grens {1000*m_advies:.0f} g", ha="right",
         fontsize=7.5, color=C["red"])
ax2.set_yticks([]); ax2.set_xlabel("massa [g]")
ax2.set_title("massabudget")
ax2.set_xlim(0, 1000 * m_advies * 1.06); ax2.set_ylim(-0.45, 0.45)
ax2.grid(axis="y", visible=False)
save(fig, "f8_massa")

# ==========================================================================
#  Figuur 9 -- verticale opbouw
# ==========================================================================
fig, (axL, axR) = plt.subplots(1, 2, figsize=(7.4, 3.1), width_ratios=[1.45, 1],
                               layout="constrained")

axL.axhspan(-6, 0, color="#cfcfcf")
axL.axhline(0, color="#555555", lw=1.2)
axL.add_patch(Circle((0, z_axle), r_w, fc=C["blue"], alpha=0.16, ec=C["blue"], lw=1.2))
axL.add_patch(Circle((-d_opt, 9), 9, fc=C["orange"], alpha=0.30, ec=C["orange"], lw=1.2))

blocks = [
    ("motor",       -52, 18, z_mot_lo, z_mot_hi, C["grey"]),
    ("hoofddek",    -90, 54, z_deck1 - h_deck_plate, z_deck1, "#9a9a9a"),
    ("accu 4xAA", -85.5, -24.5, z_deck1, z_deck1 + h_accu, C["green"]),
    ("Xplained",  -25.5, 49.5, z_deck1, z_deck1 + h_xplained, C["purple"]),
    ("bovendek",    -80, 54, z_deck2 - h_deck_plate, z_deck2, "#9a9a9a"),
    ("breadboard",  -72, 12, z_deck2, z_deck2 + h_breadboard, C["orange"]),
    ("L298N",         9, 52, z_deck2, z_top, C["teal"]),
    ("HC-SR04",      58, 70, z_sr04 - 10, z_sr04 + 10, C["red"]),
    ("lijnsensor",   48, 70, 5, 11, C["red"]),
]
for name, x0, x1, z0, z1, col in blocks:
    axL.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc=col, alpha=0.30,
                            ec=col, lw=1.1, zorder=3))
    if z1 - z0 < 5:                       # dunne plaat: label ernaast
        axL.text(x1 + 3, (z0 + z1) / 2, name, fontsize=6.0, ha="left", va="center",
                 color="#1a1a1a", zorder=6)
    else:
        axL.text((x0 + x1) / 2, (z0 + z1) / 2, name, fontsize=6.0, ha="center",
                 va="center", color="#1a1a1a", zorder=6)

axL.plot(-x_g, h_cg, "o", ms=7, color=C["red"], zorder=7)
axL.text(-x_g - 5, h_cg, f" $h$ = {h_cg:.0f}", fontsize=7, color=C["red"], ha="right", va="center")
axL.axhline(h_max, color=C["red"], lw=1.2, ls="--")
axL.text(-138, h_max + 3.5, f"grens $h$ = {h_max:.0f} mm", fontsize=6.5, color=C["red"], ha="left")

for z, lab in ((z_axle, f"as {z_axle:.0f}"), (z_deck1, f"hoofddek {z_deck1:.0f}"),
               (z_deck2, f"bovendek {z_deck2:.0f}"), (z_top, f"top {z_top:.0f}")):
    axL.plot([-112, -104], [z, z], color="#777777", lw=0.7)
    axL.text(-114, z, lab, fontsize=6.0, ha="right", va="center", color="#555555")

axL.set_xlim(-140, 72); axL.set_ylim(-6, 100); axL.set_aspect("equal")
axL.set_xlabel("x [mm]"); axL.set_ylabel("z [mm]")
axL.set_title("verticale opbouw, op schaal")
axL.grid(alpha=0.15)

mt = np.linspace(0, 400, 300) / 1000.0
h_of = (Sum_mz + mt * (z_top - 5.0)) / (m_tot + mt)
axR.plot(1000 * mt, h_of, color=C["purple"], lw=1.8)
axR.axhline(h_max_pitch, color=C["red"], lw=1.3)
axR.text(8, h_max_pitch + 0.7, f"kantelen: $h \\leq x_g/0{{,}}4 = {h_max_pitch:.0f}$ mm",
         fontsize=6.5, color=C["red"])
axR.axhline(h_max_roll, color=C["grey"], lw=1.1, ls=":")
axR.text(8, h_max_roll + 0.7, f"zijdelings: $h \\leq b/2\\mu = {h_max_roll:.0f}$ mm",
         fontsize=6.5, color=C["grey"])
axR.axvline(1000 * m_top_max, color=C["green"], lw=1.3, ls="--")
axR.plot(1000 * m_top_max, h_max_pitch, "o", ms=6, color=C["green"])
axR.annotate(f"{1000*m_top_max:.0f} g", xy=(1000 * m_top_max, h_max_pitch),
             xytext=(1000 * m_top_max - 30, h_max_pitch - 9), fontsize=7.5, color=C["green"],
             arrowprops=dict(arrowstyle="->", color=C["green"], lw=0.9))
axR.plot(0, h_cg, "o", ms=6, color=C["blue"])
axR.text(6, h_cg - 2.5, f"nu {h_cg:.0f} mm", fontsize=7, color=C["blue"])
axR.set_xlabel("extra massa op het bovendek [g]")
axR.set_ylabel("hoogte zwaartepunt $h$ [mm]")
axR.set_title("hoeveel je bovenop mag stapelen")
axR.set_xlim(0, 400); axR.set_ylim(35, 80)
save(fig, "f9_hoogte")

# ==========================================================================
# LaTeX-macronamen mogen geen cijfers bevatten
DIGIT = str.maketrans({"0": "Nul", "1": "Een", "2": "Twee", "3": "Drie", "4": "Vier",
                       "5": "Vijf", "6": "Zes", "7": "Zeven", "8": "Acht", "9": "Negen"})
with open(os.path.join(HERE, "resultaten.tex"), "w", encoding="utf-8") as f:
    f.write("% automatisch gegenereerd door analyse.py -- niet met de hand aanpassen\n")
    for k, v in out.items():
        # Nederlandse decimale komma; {,} houdt de spatiëring goed in wiskundemodus
        f.write("\\newcommand{\\R%s}{%s}\n" % (k.translate(DIGIT), v.replace(".", "{,}")))

print("figuren, massastaat.tex en resultaten.tex geschreven")
for k, v in out.items():
    print(f"  {k:14s} = {v}")
