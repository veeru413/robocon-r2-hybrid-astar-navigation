"""
R2 Robot Navigation Algorithm for ABU Robocon 2026
Meihua Forest Traversal with Remote KFS Detection
Flow: SCAN & DETECT TYPE (together) → CLIMB only if R2 KFS
"""

import pygame
import time
from enum import Enum
from typing import List, Tuple, Optional, Set
import heapq
from dataclasses import dataclass, field

# Constants
BLOCK_SIZE = 60  # pixels for visualization
GRID_COLS = 3
GRID_ROWS = 4
TIME_LIMIT = 90  # 1.5 minutes in seconds
CLIMB_TIME = 10  # seconds
SCAN_DETECT_TIME = 2  # seconds - scan AND detect type together (remote sensing)
PICKUP_TIME = 10  # seconds

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN_200 = (41, 82, 16)
GREEN_400 = (42, 113, 56)
GREEN_600 = (152, 166, 80)
RED = (223, 34, 34)
BLUE = (50, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (128, 128, 128)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)

class KFSType(Enum):
    NONE = 0
    R1_KFS = 1
    R2_KFS = 2
    FAKE_KFS = 3

class BlockHeight(Enum):
    H_200 = 200
    H_400 = 400
    H_600 = 600

@dataclass
class Block:
    """Represents a block in the Meihua Forest"""
    row: int
    col: int
    height: int  # in mm (200, 400, or 600)
    kfs_type: KFSType = KFSType.NONE
    visited: bool = False
    detected_type: Optional[KFSType] = None  # What robot learned remotely
    
    def __hash__(self):
        return hash((self.row, self.col))
    
    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

@dataclass(order=True)
class PriorityNode:
    """Node for priority queue in A* search"""
    priority: float
    block: Block = field(compare=False)
    path: List[Block] = field(compare=False, default_factory=list)

class MeihuaForest:
    """Represents the Meihua Forest grid"""
    
    def __init__(self):
        self.grid = self._initialize_grid()
        self.entrance_blocks = [(0, 0), (0, 1), (0, 2)]  # Blocks 1, 2, 3
        self.exit_blocks = [(3, 0), (3, 1), (3, 2)]  # Blocks 10, 11, 12
        
    def _initialize_grid(self) -> List[List[Block]]:
        """Initialize the 4x3 grid with block heights"""
        heights = [
            [400, 200, 400],  # Row 0: blocks 1, 2, 3
            [200, 400, 600],  # Row 1: blocks 4, 5, 6
            [400, 600, 400],  # Row 2: blocks 7, 8, 9
            [200, 400, 200]   # Row 3: blocks 10, 11, 12
        ]
        
        grid = []
        for row in range(GRID_ROWS):
            grid_row = []
            for col in range(GRID_COLS):
                block = Block(row=row, col=col, height=heights[row][col])
                grid_row.append(block)
            grid.append(grid_row)
        
        return grid
    
    def place_kfs_randomly(self, config: dict):
        """Place KFS objects based on opponent configuration"""
        for (row, col), kfs_type in config.items():
            if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
                self.grid[row][col].kfs_type = kfs_type
    
    def get_block(self, row: int, col: int) -> Optional[Block]:
        """Get block at given position"""
        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            return self.grid[row][col]
        return None
    
    def get_neighbors(self, block: Block) -> List[Block]:
        """Get valid neighboring blocks (within climbable height difference)"""
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        for dr, dc in directions:
            new_row, new_col = block.row + dr, block.col + dc
            neighbor = self.get_block(new_row, new_col)
            
            if neighbor:
                height_diff = abs(neighbor.height - block.height)
                if height_diff <= 200:
                    neighbors.append(neighbor)
        
        return neighbors

class R2Robot:
    """R2 Robot with remote KFS detection capabilities"""
    
    def __init__(self, forest: MeihuaForest):
        self.forest = forest
        self.current_block = None
        self.collected_kfs = []
        self.time_elapsed = 0.0
        self.path_history = []
        self.detection_history = []
        
    def heuristic(self, block: Block, goal_blocks: List[Tuple[int, int]]) -> float:
        """Heuristic: minimum Manhattan distance to any goal block"""
        min_dist = float('inf')
        for goal_row, goal_col in goal_blocks:
            dist = abs(block.row - goal_row) + abs(block.col - goal_col)
            min_dist = min(min_dist, dist)
        return min_dist
    
    def scan_and_detect_adjacent(self, current: Block) -> List[Tuple[Block, KFSType]]:
        """
        SCAN & DETECT: Remotely check adjacent blocks and determine KFS type
        Uses camera/sensor to identify type WITHOUT climbing
        Time: 2 seconds per adjacent block
        Returns: list of (block, kfs_type) tuples
        """
        detected = []
        neighbors = self.forest.get_neighbors(current)
        
        for neighbor in neighbors:
            if neighbor.detected_type is None:  # Haven't scanned this block yet
                if self.time_elapsed + SCAN_DETECT_TIME > TIME_LIMIT:
                    break
                
                self.time_elapsed += SCAN_DETECT_TIME
                # Remote detection - can see and identify KFS type without climbing
                detected_type = neighbor.kfs_type
                neighbor.detected_type = detected_type
                detected.append((neighbor, detected_type))
                self.detection_history.append((neighbor.row, neighbor.col, detected_type))
        
        return detected
    
    def climb_to_block(self, target: Block) -> bool:
        """Climb to target block (10 seconds)"""
        if self.time_elapsed + CLIMB_TIME > TIME_LIMIT:
            return False
        
        self.time_elapsed += CLIMB_TIME
        self.current_block = target
        target.visited = True
        self.path_history.append((target.row, target.col))
        return True
    
    def pickup_kfs(self, block: Block) -> bool:
        """Pick up R2 KFS from current block (10 seconds)"""
        if self.time_elapsed + PICKUP_TIME > TIME_LIMIT:
            return False
        
        if block.kfs_type == KFSType.R2_KFS:
            self.time_elapsed += PICKUP_TIME
            self.collected_kfs.append((block.row, block.col))
            block.kfs_type = KFSType.NONE
            return True
        return False
    
    def find_path_astar(self, start: Block, goal_blocks: List[Tuple[int, int]]) -> Optional[List[Block]]:
        """A* pathfinding to reach any goal block"""
        frontier = []
        heapq.heappush(frontier, PriorityNode(0, start, [start]))
        
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while frontier:
            current_node = heapq.heappop(frontier)
            current = current_node.block
            
            if (current.row, current.col) in goal_blocks:
                return current_node.path
            
            for neighbor in self.forest.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal_blocks)
                    new_path = current_node.path + [neighbor]
                    heapq.heappush(frontier, PriorityNode(priority, neighbor, new_path))
                    came_from[neighbor] = current
        
        return None
    
    def navigate_forest(self, start_pos: Tuple[int, int]) -> dict:
        """
        Main navigation algorithm: SCAN+DETECT → CLIMB ONLY IF R2 KFS
        
        Algorithm Flow:
        1. Start at entrance block
        2. SCAN & DETECT adjacent blocks (2s each) - identify KFS type remotely
        3. If R2 KFS found → add to priority queue
        4. CLIMB to nearest R2 KFS location
        5. PICKUP R2 KFS (10s)
        6. Repeat from new position
        7. If no R2 KFS nearby → explore unvisited blocks systematically
        """
        start_block = self.forest.get_block(*start_pos)
        self.current_block = start_block
        self.time_elapsed = 0.0
        start_block.visited = True
        self.path_history.append((start_block.row, start_block.col))
        
        # Priority queue: blocks confirmed to have R2 KFS
        r2_kfs_blocks = []
        visited_positions = set()
        visited_positions.add((start_block.row, start_block.col))
        
        # Check if starting block has R2 KFS (scan it)
        if start_block.kfs_type != KFSType.NONE:
            if self.time_elapsed + SCAN_DETECT_TIME <= TIME_LIMIT:
                self.time_elapsed += SCAN_DETECT_TIME
                start_block.detected_type = start_block.kfs_type
                self.detection_history.append((start_block.row, start_block.col, start_block.kfs_type))
                
                if start_block.kfs_type == KFSType.R2_KFS:
                    self.pickup_kfs(start_block)
        
        # Initial scan & detect from starting position
        detected = self.scan_and_detect_adjacent(start_block)
        for block, kfs_type in detected:
            if kfs_type == KFSType.R2_KFS:
                heapq.heappush(r2_kfs_blocks, (
                    self.heuristic(block, [(start_block.row, start_block.col)]),
                    (block.row, block.col)
                ))
        
        # Main exploration loop
        while self.time_elapsed < TIME_LIMIT:
            # Priority 1: Collect detected R2 KFS
            if r2_kfs_blocks:
                _, target_pos = heapq.heappop(r2_kfs_blocks)
                target_block = self.forest.get_block(*target_pos)
                
                if target_pos in visited_positions:
                    continue
                
                # Navigate to R2 KFS location
                path = self.find_path_astar(self.current_block, [target_pos])
                
                if path:
                    # Traverse path to reach R2 KFS
                    for block in path[1:]:
                        if not self.climb_to_block(block):
                            return self._get_results()
                        
                        visited_positions.add((block.row, block.col))
                        
                        # Scan & detect from new position
                        detected = self.scan_and_detect_adjacent(block)
                        for detected_block, kfs_type in detected:
                            pos = (detected_block.row, detected_block.col)
                            if kfs_type == KFSType.R2_KFS and pos not in visited_positions:
                                heapq.heappush(r2_kfs_blocks, (
                                    self.heuristic(detected_block, [(block.row, block.col)]),
                                    pos
                                ))
                        
                        # If this block has R2 KFS, pick it up
                        if block.kfs_type == KFSType.R2_KFS and block.detected_type == KFSType.R2_KFS:
                            if not self.pickup_kfs(block):
                                return self._get_results()
                    
                    # Pick up KFS at target block
                    if self.current_block.kfs_type == KFSType.R2_KFS:
                        if not self.pickup_kfs(self.current_block):
                            return self._get_results()
            
            else:
                # Priority 2: Systematic exploration to find more R2 KFS
                neighbors = self.forest.get_neighbors(self.current_block)
                unvisited = [n for n in neighbors if (n.row, n.col) not in visited_positions]
                
                if not unvisited:
                    # Dead end - find unvisited regions
                    all_unvisited = []
                    for r in range(GRID_ROWS):
                        for c in range(GRID_COLS):
                            if (r, c) not in visited_positions:
                                block = self.forest.get_block(r, c)
                                path = self.find_path_astar(self.current_block, [(r, c)])
                                if path:
                                    all_unvisited.append(((r, c), len(path)))
                    
                    if not all_unvisited:
                        break  # All reachable blocks explored
                    
                    # Go to nearest unvisited
                    target_pos, _ = min(all_unvisited, key=lambda x: x[1])
                    path = self.find_path_astar(self.current_block, [target_pos])
                    
                    if path:
                        for step in path[1:]:
                            if not self.climb_to_block(step):
                                return self._get_results()
                            visited_positions.add((step.row, step.col))
                            
                            # Scan & detect from new position
                            detected = self.scan_and_detect_adjacent(step)
                            for detected_block, kfs_type in detected:
                                pos = (detected_block.row, detected_block.col)
                                if kfs_type == KFSType.R2_KFS and pos not in visited_positions:
                                    heapq.heappush(r2_kfs_blocks, (
                                        self.heuristic(detected_block, [(step.row, step.col)]),
                                        pos
                                    ))
                            
                            # If R2 KFS on this block, pick it up
                            if step.kfs_type == KFSType.R2_KFS and step.detected_type == KFSType.R2_KFS:
                                if not self.pickup_kfs(step):
                                    return self._get_results()
                    else:
                        break
                
                else:
                    # Visit nearest unvisited neighbor
                    next_block = min(unvisited, key=lambda b: 
                        self.heuristic(b, [(self.current_block.row, self.current_block.col)]))
                    
                    if not self.climb_to_block(next_block):
                        break
                    
                    visited_positions.add((next_block.row, next_block.col))
                    
                    # Scan & detect from new position
                    detected = self.scan_and_detect_adjacent(next_block)
                    for detected_block, kfs_type in detected:
                        pos = (detected_block.row, detected_block.col)
                        if kfs_type == KFSType.R2_KFS and pos not in visited_positions:
                            heapq.heappush(r2_kfs_blocks, (
                                self.heuristic(detected_block, [(next_block.row, next_block.col)]),
                                pos
                            ))
                    
                    # If R2 KFS on current block, pick it up
                    if next_block.kfs_type == KFSType.R2_KFS and next_block.detected_type == KFSType.R2_KFS:
                        if not self.pickup_kfs(next_block):
                            return self._get_results()
        
        return self._get_results()
    
    def _get_results(self) -> dict:
        """Package navigation results"""
        return {
            'time_elapsed': self.time_elapsed,
            'collected_kfs': self.collected_kfs,
            'path_history': self.path_history,
            'detection_history': self.detection_history,
            'success': self.time_elapsed <= TIME_LIMIT,
            'kfs_count': len(self.collected_kfs)
        }


class NavigationVisualizer:
    """Pygame-based visualization of robot navigation"""
    
    def __init__(self, forest: MeihuaForest, robot: R2Robot):
        pygame.init()
        
        self.forest = forest
        self.robot = robot
        
        self.screen_width = 900
        self.screen_height = 750
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("R2 Robot - Remote Scan+Detect Navigation")
        
        self.grid_offset_x = 350
        self.grid_offset_y = 300
        
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
    def get_block_color(self, block: Block) -> Tuple[int, int, int]:
        """Get color based on block height"""
        if block.height == 200:
            return GREEN_200
        elif block.height == 400:
            return GREEN_400
        else:
            return GREEN_600
    
    def draw_grid(self):
        """Draw the Meihua Forest grid"""
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                block = self.forest.grid[row][col]
                
                x = self.grid_offset_x + col * (BLOCK_SIZE + 10)
                y = self.grid_offset_y + row * (BLOCK_SIZE + 10)
                
                color = self.get_block_color(block)
                pygame.draw.rect(self.screen, color, (x, y, BLOCK_SIZE, BLOCK_SIZE))
                pygame.draw.rect(self.screen, BLACK, (x, y, BLOCK_SIZE, BLOCK_SIZE), 2)
                
                # Height label
                height_text = self.small_font.render(f"{block.height}H", True, WHITE)
                self.screen.blit(height_text, (x + 5, y + 5))
                
                # Block number
                block_num = row * 3 + col + 1
                num_text = self.font.render(str(block_num), True, WHITE)
                self.screen.blit(num_text, (x + BLOCK_SIZE - 20, y + BLOCK_SIZE - 25))
                
                # KFS indicator (actual placement)
                if block.kfs_type != KFSType.NONE:
                    kfs_color = RED if block.kfs_type == KFSType.R1_KFS else BLUE if block.kfs_type == KFSType.R2_KFS else YELLOW
                    pygame.draw.circle(self.screen, kfs_color, (x + BLOCK_SIZE // 2, y + BLOCK_SIZE // 2), 15)
                    kfs_text = self.small_font.render("KFS", True, BLACK)
                    self.screen.blit(kfs_text, (x + BLOCK_SIZE // 2 - 12, y + BLOCK_SIZE // 2 - 6))
                
                # Visited indicator (climbed)
                if block.visited:
                    pygame.draw.rect(self.screen, ORANGE, (x, y, BLOCK_SIZE, BLOCK_SIZE), 3)
                
                # Detected indicator (scanned remotely)
                if block.detected_type is not None:
                    pygame.draw.circle(self.screen, PURPLE, (x + 10, y + 10), 5)
    
    def draw_robot(self):
        """Draw robot at current position"""
        if self.robot.current_block:
            block = self.robot.current_block
            x = self.grid_offset_x + block.col * (BLOCK_SIZE + 10) + BLOCK_SIZE // 2
            y = self.grid_offset_y + block.row * (BLOCK_SIZE + 10) + BLOCK_SIZE // 2
            
            pygame.draw.circle(self.screen, RED, (x, y), 12)
            pygame.draw.circle(self.screen, WHITE, (x, y), 12, 2)
            robot_text = self.small_font.render("R2", True, WHITE)
            self.screen.blit(robot_text, (x - 8, y - 6))
    
    def draw_stats(self, results: dict = None):
        """Draw statistics panel"""
        stats_x = 20
        stats_y = 20
        
        title = self.font.render("Navigation Statistics", True, BLACK)
        self.screen.blit(title, (stats_x, stats_y))
        
        y_offset = stats_y + 30
        
        if results:
            time_text = self.font.render(f"Time: {results['time_elapsed']:.1f}s / {TIME_LIMIT}s", True, BLACK)
            self.screen.blit(time_text, (stats_x, y_offset))
            y_offset += 25
            
            kfs_text = self.font.render(f"R2 KFS Collected: {results['kfs_count']}", True, BLACK)
            self.screen.blit(kfs_text, (stats_x, y_offset))
            y_offset += 25
            
            visited_text = self.font.render(f"Blocks Visited: {len(results['path_history'])}", True, BLACK)
            self.screen.blit(visited_text, (stats_x, y_offset))
            y_offset += 25
            
            detected_text = self.font.render(f"Remote Detections: {len(results['detection_history'])}", True, BLACK)
            self.screen.blit(detected_text, (stats_x, y_offset))
            y_offset += 25
            
            status = "SUCCESS" if results['success'] else "TIME LIMIT EXCEEDED"
            status_color = GREEN_400 if results['success'] else RED
            status_text = self.font.render(f"Status: {status}", True, status_color)
            self.screen.blit(status_text, (stats_x, y_offset))
    
    def draw_legend(self):
        """Draw legend for visualization"""
        legend_x = self.screen_width - 230
        legend_y = 100
        
        legend_title = self.font.render("Legend", True, BLACK)
        self.screen.blit(legend_title, (legend_x, legend_y))
        
        y_offset = legend_y + 30
        
        legends = [
            (RED, "R1 KFS (ignored)"),
            (BLUE, "R2 KFS (collected)"),
            (YELLOW, "Fake KFS (ignored)"),
            (ORANGE, "Visited (climbed)"),
            (PURPLE, "Detected (remotely)"),
        ]
        
        for color, text in legends:
            if color == PURPLE:
                pygame.draw.circle(self.screen, color, (legend_x + 10, y_offset + 8), 6)
            else:
                pygame.draw.circle(self.screen, color, (legend_x + 10, y_offset + 8), 8)
            legend_text = self.small_font.render(text, True, BLACK)
            self.screen.blit(legend_text, (legend_x + 25, y_offset))
            y_offset += 25
    
    def visualize_navigation(self, results: dict):
        """Run visualization of navigation results"""
        clock = pygame.time.Clock()
        running = True
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            self.screen.fill(WHITE)
            
            self.draw_grid()
            self.draw_robot()
            self.draw_stats(results)
            self.draw_legend()
            
            instr_text = self.small_font.render("Press ESC to close", True, GRAY)
            self.screen.blit(instr_text, (self.screen_width // 2 - 60, self.screen_height - 30))
            
            pygame.display.flip()
            clock.tick(60)
        
        pygame.quit()


def main():
    """Main function to run the simulation"""
    
    forest = MeihuaForest()
    
    # Example KFS configuration (opponent placement)
    kfs_config = {
        (0, 1): KFSType.FAKE_KFS,  # Block 2 (starting position)
        (1, 1): KFSType.R2_KFS,    # Block 5
        (1, 2): KFSType.R2_KFS,    # Block 6
        (2, 0): KFSType.R1_KFS,    # Block 7
        (2, 1): KFSType.R2_KFS,    # Block 8
        (3, 2): KFSType.R2_KFS,    # Block 12
    }
    
    forest.place_kfs_randomly(kfs_config)
    
    robot = R2Robot(forest)
    
    print("="*60)
    print("R2 ROBOT NAVIGATION SIMULATION")
    print("ABU Robocon 2026 - Meihua Forest")
    print("="*60)
    print(f"\nTime limit: {TIME_LIMIT} seconds")
    print(f"KFS Configuration:")
    print(f"  - R2 KFS: {len([k for k in kfs_config.values() if k == KFSType.R2_KFS])} placed")
    print(f"  - R1 KFS: {len([k for k in kfs_config.values() if k == KFSType.R1_KFS])} placed")
    print(f"  - Fake KFS: {len([k for k in kfs_config.values() if k == KFSType.FAKE_KFS])} placed")
    print("\nAlgorithm: SCAN+DETECT (2s remote) → CLIMB ONLY IF R2 KFS")
    print("Robot scans adjacent blocks and detects type WITHOUT climbing")
    print("Only climbs to blocks confirmed to have R2 KFS\n")
    print("Navigating...\n")
    
    results = robot.navigate_forest(start_pos=(0, 1))
    
    print("="*60)
    print("NAVIGATION RESULTS")
    print("="*60)
    print(f"\nTime Performance:")
    print(f"  Total Time: {results['time_elapsed']:.2f}s")
    print(f"  Time Remaining: {TIME_LIMIT - results['time_elapsed']:.2f}s")
    print(f"  Status: {'✓ SUCCESS' if results['success'] else '✗ TIMEOUT'}")
    
    print(f"\nCollection Performance:")
    print(f"  R2 KFS Collected: {results['kfs_count']}")
    print(f"  Collection Points: {results['collected_kfs']}")
    
    print(f"\nExploration Statistics:")
    print(f"  Blocks Visited: {len(results['path_history'])}")
    print(f"  Remote Detections: {len(results['detection_history'])}")
    
    print(f"\nPath Taken:")
    print(f"  {' → '.join([f'Block {r*3+c+1}' for r, c in results['path_history']])}")
    
    print(f"\nDetection History (Remote Sensing):")
    r2_found = 0
    r1_found = 0
    fake_found = 0
    for row, col, kfs_type in results['detection_history']:
        block_num = row * 3 + col + 1
        type_name = kfs_type.name.replace('_', ' ')
        print(f"  Block {block_num}: {type_name}")
        if kfs_type == KFSType.R2_KFS:
            r2_found += 1
        elif kfs_type == KFSType.R1_KFS:
            r1_found += 1
        elif kfs_type == KFSType.FAKE_KFS:
            fake_found += 1
    
    print(f"\nDetection Summary:")
    print(f"  R2 KFS detected: {r2_found}")
    print(f"  R1 KFS detected: {r1_found} (ignored)")
    print(f"  Fake KFS detected: {fake_found} (ignored)")
    print(f"  Empty blocks detected: {len(results['detection_history']) - r2_found - r1_found - fake_found}")
    
    print(f"\nTime Breakdown (Estimated):")
    climb_time = (len(results['path_history']) - 1) * CLIMB_TIME
    detect_time = len(results['detection_history']) * SCAN_DETECT_TIME
    pickup_time = results['kfs_count'] * PICKUP_TIME
    
    print(f"  Remote Detection: {detect_time:.1f}s ({len(results['detection_history'])} blocks × 2s)")
    print(f"  Climbing: {climb_time:.1f}s ({len(results['path_history'])-1} transitions × 10s)")
    print(f"  Pickup: {pickup_time:.1f}s ({results['kfs_count']} R2 KFS × 10s)")
    print(f"  Total: {detect_time + climb_time + pickup_time:.1f}s")
    
    print(f"\nEfficiency Analysis:")
    total_blocks = GRID_ROWS * GRID_COLS
    detected_percentage = (len(results['detection_history']) / total_blocks) * 100
    visited_percentage = (len(results['path_history']) / total_blocks) * 100
    print(f"  Blocks detected remotely: {len(results['detection_history'])}/{total_blocks} ({detected_percentage:.1f}%)")
    print(f"  Blocks actually visited: {len(results['path_history'])}/{total_blocks} ({visited_percentage:.1f}%)")
    print(f"  Climb savings: {detected_percentage - visited_percentage:.1f}% of blocks scanned without climbing")
    
    print("\n" + "="*60)
    print("Launching visualization...")
    print("="*60)
    
    visualizer = NavigationVisualizer(forest, robot)
    visualizer.visualize_navigation(results)


if __name__ == "__main__":
    main()