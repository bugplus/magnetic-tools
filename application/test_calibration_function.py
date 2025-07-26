#!/usr/bin/env python3
# diff_cal.py
import re, sys, numpy as np

def parse(path):
    with open(path) as f:
        txt = f.read()
    # 抓取 HARD_IRON 行
    h_str = re.search(r'HARD_IRON\[3\] = {(.*?)}', txt, re.S).group(1)
    b = np.array([float(x.strip().replace('f', '')) for x in h_str.split(',')])
    # 抓取 SOFT_IRON 3x3 块
    m = re.search(r'SOFT_IRON\[3\]\[3\] = {(.*?)};', txt, re.S)
    rows = re.findall(r'\{(.*?)\}', m.group(1))
    A = np.array([[float(v.strip().replace('f', '')) for v in r.split(',')] for r in rows])
    return b, A

b1, A1 = parse(sys.argv[1])
b2, A2 = parse(sys.argv[2])

print('\n--- HARD IRON ---')
print('b1:', b1)
print('b2:', b2)
print('Δb (abs):', np.abs(b1 - b2))
print('Δb (µT): ', np.linalg.norm(b1 - b2))

print('\n--- SOFT IRON ---')
print('A1:\n', A1)
print('A2:\n', A2)
print('ΔA (max abs):', np.max(np.abs(A1 - A2)))
print('ΔA (Fro):    ', np.linalg.norm(A1 - A2, 'fro'))

# 相对误差百分比
if np.linalg.norm(b1) > 0:
    print('\nRelative error (b): %.2f %%' % (100 * np.linalg.norm(b1 - b2) / np.linalg.norm(b1)))
if np.linalg.norm(A1) > 0:
    print('Relative error (A): %.2f %%' % (100 * np.linalg.norm(A1 - A2, 'fro') / np.linalg.norm(A1, 'fro')))