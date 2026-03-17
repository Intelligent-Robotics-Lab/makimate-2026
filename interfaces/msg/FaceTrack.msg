# A single tracked face with its attention priority score.
int32 id                  # Persistent tracking ID across frames
string name               # Identity from facial recognition (default: "Unknown")
int32[4] bbox             # Bounding box [x, y, w, h] in pixels
float32 confidence_score  # Attention priority score in [0.0, 1.0]
bool is_speaking          # True if lip movement variance exceeds threshold
