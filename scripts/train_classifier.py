#!/usr/bin/env python3
"""
train_classifier.py

Trains classifier for three distinct Gazebo plant foliage colors:
  healthy_ripe   (Red):    Gazebo/Red    -> H ~ 0 (or ~178), S ~ 245, V ~ 180
  unhealthy      (Yellow): Gazebo/Yellow -> H ~ 30,          S ~ 245, V ~ 180
  healthy_unripe (Green):  Gazebo/Green  -> H ~ 60,          S ~ 245, V ~ 180

Uses hue normalization (h if h < 90 else h - 180) to handle the 0/180 degree
wraparound for red, placing the categories into distinct monotonic hue clusters:
  Red:    H_adj ~ 0
  Yellow: H_adj ~ 30
  Green:  H_adj ~ 60
"""
import os
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import pickle
import rospkg

np.random.seed(42)

SAMPLES = 500

def normalize_hue(h):
    return h if h < 90.0 else h - 180.0

# Calibrated cluster centers (H_adj, S, V)
CATEGORY_HSV = {
    "healthy_ripe":   (0.0,  245.0, 180.0),
    "unhealthy":      (30.0, 245.0, 180.0),
    "healthy_unripe": (60.0, 245.0, 180.0),
}

NOISE = {
    "healthy_ripe":   (4.0, 10.0, 20.0),
    "unhealthy":      (4.0, 10.0, 20.0),
    "healthy_unripe": (4.0, 10.0, 20.0),
}

X, y = [], []
for cat, (h, s, v) in CATEGORY_HSV.items():
    nh, ns, nv = NOISE[cat]
    hs = np.random.normal(h, nh, SAMPLES)
    ss = np.clip(np.random.normal(s, ns, SAMPLES), 40, 255)
    vs = np.clip(np.random.normal(v, nv, SAMPLES), 40, 255)
    for i in range(SAMPLES):
        X.append([hs[i], ss[i], vs[i]])
        y.append(cat)

X = np.array(X)
y = np.array(y)

X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42, stratify=y)

scaler = StandardScaler()
X_train_s = scaler.fit_transform(X_train)
X_test_s  = scaler.transform(X_test)

model = LogisticRegression(max_iter=2000, C=1.0)
model.fit(X_train_s, y_train)

print(f"Training accuracy: {model.score(X_train_s, y_train):.3f}")
print(f"Test accuracy:     {model.score(X_test_s,  y_test):.3f}")

try:
    pkg_path = rospkg.RosPack().get_path('farm_field_scout')
    out_path = os.path.join(pkg_path, 'config', 'plant_classifier.pkl')
except Exception:
    out_path = 'config/plant_classifier.pkl'

with open(out_path, "wb") as f:
    pickle.dump({'model': model, 'scaler': scaler}, f)

print(f"Saved model + scaler to {out_path}")

