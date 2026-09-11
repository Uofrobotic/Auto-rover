import serial
import math
import pygame
import sys

# ⚙️ Configurations
ARDUINO_PORT = "COM7"  
BAUD_RATE = 115200
MAX_DISTANCE = 2000

# Window dimensions
WIDTH, HEIGHT = 800, 800
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2
SCALE = (WIDTH // 2) / MAX_DISTANCE 

# Initialize Pygame Screen
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Real-Time LIDAR")
# Increased frame rate limit to allow maximum rendering speed
clock = pygame.time.Clock()

print("Connecting to Arduino...")
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.001)
    ser.reset_input_buffer() 
    print(f"✅ Secure Connection Restored on {ARDUINO_PORT}")
except Exception as e:
    print(f"❌ Connection error: {e}")
    sys.exit()

# 🚀 HIGH-PERFORMANCE VARIABLES
byte_accumulator = bytearray()
NUM_STEPS = 6000  
outline_map = [None] * NUM_STEPS

print("Launching Renderer...")
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            ser.close()
            pygame.quit()
            sys.exit()

    # 1. Clear background screen canvas
    screen.fill((5, 8, 5))

    # 2. Draw Stable Radar Grid Rings
    for r in [500, 1000, 1500, MAX_DISTANCE]:
        pygame.draw.circle(screen, (20, 35, 20), (CENTER_X, CENTER_Y), int(r * SCALE), 1)
    pygame.draw.line(screen, (20, 35, 20), (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), 1)
    pygame.draw.line(screen, (20, 35, 20), (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), 1)

    # 3. Read ALL raw stream data at once (Prevents lag buildup)
    if ser.is_open and ser.in_waiting > 0:
        byte_accumulator.extend(ser.read(ser.in_waiting))

    # 4. ⚡ HIGH-SPEED PARSING: Scan using an index pointer instead of deleting bytes
    idx = 0
    length = len(byte_accumulator)
    
    while length - idx >= 5:
        if byte_accumulator[idx] == 0xAA:
            angle_scaled = (byte_accumulator[idx+1] << 8) | byte_accumulator[idx+2]
            distance = (byte_accumulator[idx+3] << 8) | byte_accumulator[idx+4]
            
            angle = angle_scaled / 100.0 
            step_idx = int((angle % 360) * (NUM_STEPS / 360.0)) % NUM_STEPS
            
            # 🧹 FAST WIPER: Only clear a tight window to erase "ghosts" of skipped angles
            for i in range(1, 6):
                outline_map[(step_idx + i) % NUM_STEPS] = None
                outline_map[(step_idx - i) % NUM_STEPS] = None

            # Register valid point
            if 0 < distance <= MAX_DISTANCE:
                rad = math.radians(-angle)
                x = int(CENTER_X + distance * SCALE * math.cos(rad))
                y = int(CENTER_Y + distance * SCALE * math.sin(rad))
                outline_map[step_idx] = (x, y)
            else:
                outline_map[step_idx] = None
            
            idx += 5  # Jump forward 5 bytes
        else:
            idx += 1  # Slide forward 1 byte to find next 0xAA header

    # 5. Trim processed bytes from memory (Runs instantly once per frame)
    if idx > 0:
        byte_accumulator = byte_accumulator[idx:]

    # 6. Extract valid points & Render Layers
    valid_polygon_points = [pt for pt in outline_map if pt is not None]
    
    if valid_polygon_points:
        # Layer A: Draw faint radial lines from the center LIDAR to each detected point
        for pt in valid_polygon_points:
            pygame.draw.line(screen, (15, 65, 15), (CENTER_X, CENTER_Y), pt, 1)

    if len(valid_polygon_points) >= 3:
        # Layer B: Draw the connecting outline (Set closed=False so it doesn't draw a line through the wiper gap)
        pygame.draw.lines(screen, (100, 255, 100), False, valid_polygon_points, 2)
        
        # Layer C: Draw individual dots for the points
        for pt in valid_polygon_points:
            pygame.draw.circle(screen, (50, 200, 50), pt, 2)

    # Layer D: Center LIDAR Indicator (Red Dot)
    pygame.draw.circle(screen, (255, 50, 50), (CENTER_X, CENTER_Y), 4)       
    pygame.draw.circle(screen, (255, 255, 255), (CENTER_X, CENTER_Y), 5, 1)  

    pygame.display.flip()
    
    # Increased FPS cap to 120 so the visualization never limits the data flow
    clock.tick(120)