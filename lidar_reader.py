import serial
import math
import pygame
import sys

ARDUINO_PORT = "COM8"  
BAUD_RATE = 500000 
MAX_DISTANCE = 5000  

WIDTH, HEIGHT = 800, 800
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2
SCALE = (WIDTH // 2) / MAX_DISTANCE 

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Real-Time LIDAR Map")
clock = pygame.time.Clock()

print("Connecting to ESP32...")
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.001)
    ser.reset_input_buffer() 
    print(f"✅ Secure Connection Restored on {ARDUINO_PORT}")
except Exception as e:
    print(f"❌ Connection error: {e}")
    sys.exit()

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

    screen.fill((10, 10, 10))

    # Draw Orange Grid Rings & Crosshairs
    for r in [1000, 2000, 3000, 4000, MAX_DISTANCE]:
        pygame.draw.circle(screen, (60, 30, 0), (CENTER_X, CENTER_Y), int(r * SCALE), 1)
    pygame.draw.line(screen, (60, 30, 0), (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), 1)
    pygame.draw.line(screen, (60, 30, 0), (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), 1)

    if ser.is_open and ser.in_waiting > 0:
        byte_accumulator.extend(ser.read(ser.in_waiting))

    idx = 0
    length = len(byte_accumulator)
    
    while length - idx >= 6:
        if byte_accumulator[idx] == 0x55 and byte_accumulator[idx+1] == 0xAA:
            angle_scaled = (byte_accumulator[idx+2] << 8) | byte_accumulator[idx+3]
            distance = (byte_accumulator[idx+4] << 8) | byte_accumulator[idx+5]
            
            angle = angle_scaled / 100.0 
            step_idx = int((angle % 360) * (NUM_STEPS / 360.0)) % NUM_STEPS
            
            for i in range(1, 8):
                outline_map[(step_idx + i) % NUM_STEPS] = None

            if 0 < distance <= MAX_DISTANCE:
                rad = math.radians(-angle)
                x = int(CENTER_X + distance * SCALE * math.cos(rad))
                y = int(CENTER_Y + distance * SCALE * math.sin(rad))
                outline_map[step_idx] = (x, y)
            else:
                outline_map[step_idx] = None
            
            idx += 6  
        else:
            idx += 1  

    if idx > 0:
        byte_accumulator = byte_accumulator[idx:]

    # Group points into continuous segments based on angular proximity (prevents webbing across gaps)
    segments = []
    current_seg = []
    last_idx = -1

    for i in range(NUM_STEPS):
        if outline_map[i] is not None:
            if last_idx == -1 or (i - last_idx) <= 5: 
                current_seg.append(outline_map[i])
            else:
                if len(current_seg) >= 2:
                    segments.append(current_seg)
                current_seg = [outline_map[i]]
            last_idx = i
        else:
            if last_idx != -1 and (i - last_idx) > 5:
                if len(current_seg) >= 2:
                    segments.append(current_seg)
                current_seg = []

    if current_seg and len(current_seg) >= 2:
        segments.append(current_seg)

    # Render distinct wall boundaries in clean white lines
    for seg in segments:
        pygame.draw.lines(screen, (255, 255, 255), False, seg, 1)

    # Render Orange Laser Rays connecting center to points, plus point markers
    for pt in [p for p in outline_map if p is not None]:
        pygame.draw.line(screen, (40, 20, 0), (CENTER_X, CENTER_Y), pt, 1)
        pygame.draw.circle(screen, (255, 140, 0), pt, 2) 

    # Center LIDAR Scanner Indicator
    pygame.draw.circle(screen, (255, 50, 50), (CENTER_X, CENTER_Y), 4)      
    pygame.draw.circle(screen, (255, 255, 255), (CENTER_X, CENTER_Y), 5, 1)  

    pygame.display.flip()
    clock.tick(0)