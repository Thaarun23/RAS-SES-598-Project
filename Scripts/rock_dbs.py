#!/usr/bin/env python3
import os, pickle

# Path to your DB
db_path = os.path.expanduser('~/rock_features.db')

# Load it
with open(db_path, 'rb') as f:
    db = pickle.load(f)

if not db:
    print("No rocks in database.")
    exit(0)

# Print header
print(f"{'ID':>3} | {'X':>8} {'Y':>8} {'Z':>8} | #Features")
print("-"*40)

# Iterate
for entry in db:
    rid = entry['id']
    x,y,z = entry['position']
    nfeat = len(entry['descriptors'])
    print(f"{rid:3d} | {x:8.3f} {y:8.3f} {z:8.3f} | {nfeat:9d}")

