import pcbnew
import math

# Get current board
board = pcbnew.GetBoard()

# Configuration
num_leds = 8
radius_mm = 40
cap_offset_mm = 5
center_mm = (100, 100)

# Convert center to internal units
center = pcbnew.VECTOR2I(
    pcbnew.FromMM(center_mm[0]),
    pcbnew.FromMM(center_mm[1])
)

# Get original LED and capacitor to clone
orig_led = board.FindFootprintByReference("D1")
orig_cap = board.FindFootprintByReference("C1")

if not orig_led or not orig_cap:
    raise Exception("Place D1 and C1 manually on the board before running this script.")

# Remove original footprints after cloning
board.Remove(orig_led)
board.Remove(orig_cap)

# Create circular outline
def create_circle_outline(board, radius_mm, segments=100):
    outline = pcbnew.PCB_SHAPE(board)
    outline.SetLayer(pcbnew.Edge_Cuts)
    outline.SetShape(pcbnew.SHAPE_POLY)
    
    points = []
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = center.x + pcbnew.FromMM(radius_mm * math.cos(angle))
        y = center.y + pcbnew.FromMM(radius_mm * math.sin(angle))
        points.append(pcbnew.VECTOR2I(int(x), int(y)))

    outline.SetPolyPoints(points)
    board.Add(outline)

# Duplicate footprints around the circle
angle_step = 360 / num_leds

for i in range(num_leds):
    angle_deg = i * angle_step
    angle_rad = math.radians(angle_deg)

    # LED
    x_led = center.x + pcbnew.FromMM(radius_mm * math.cos(angle_rad))
    y_led = center.y + pcbnew.FromMM(radius_mm * math.sin(angle_rad))
    pos_led = pcbnew.VECTOR2I(int(x_led), int(y_led))

    new_led = orig_led.Clone()
    new_led.SetReference(f"D{i+1}")
    new_led.SetPosition(pos_led)
    new_led.SetOrientationDegrees(angle_deg + 180)
    board.Add(new_led)

    # Capacitor
    x_cap = center.x + pcbnew.FromMM((radius_mm + cap_offset_mm) * math.cos(angle_rad))
    y_cap = center.y + pcbnew.FromMM((radius_mm + cap_offset_mm) * math.sin(angle_rad))
    pos_cap = pcbnew.VECTOR2I(int(x_cap), int(y_cap))

    new_cap = orig_cap.Clone()
    new_cap.SetReference(f"C{i+1}")
    new_cap.SetPosition(pos_cap)
    new_cap.SetOrientationDegrees(angle_deg)
    board.Add(new_cap)

# Outline
create_circle_outline(board, radius_mm + 6)

# Save
pcbnew.SaveBoard("circular_led_board_clone.kicad_pcb", board)
print("[✅] Board saved as circular_led_board_clone.kicad_pcb")
