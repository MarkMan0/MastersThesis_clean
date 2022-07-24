import pygame
import time
import robot_sprite


class Visualisation:

    def __init__(self):
        self.running = True
        self.window = None
        self.display_sz = (1000, 450)
        self.display = None
        self.simulation = None
        self.robot = None
        self.visited = set()

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self.running = False

    def on_render(self):
        self.window.fill((0, 0, 0))
        self.display.fill((40, 40, 40))
        self.robot.screen = self.display
        self.robot.render()
        self.draw_position()
        self.draw_path()
        self.display = pygame.transform.flip(self.display, False, True)
        rect = self.display.get_rect()
        rect.topleft = (0, 0)
        self.window.blit(self.display, rect)
        pygame.display.flip()

    def cleanup(self):
        self.simulation.stop()
        pygame.quit()

    def on_update(self):
        self.robot.set_pos(self.simulation.position)
        self.visited.add(self.robot.center)

    def tick(self):
        for event in pygame.event.get():
            self.on_event(event)
        self.on_update()
        self.on_render()

    def thread(self):
        pygame.init()
        self.font = pygame.font.SysFont('monospace', 15)
        self.running = True
        self.window = pygame.display.set_mode(size=(1000, 500))
        self.display = pygame.Surface(size=self.display_sz)
        self.robot = robot_sprite.Car(self.display)
        while self.running:
            self.tick()
            pygame.time.wait(int(1000/30))
        self.cleanup()

    def draw_position(self):
        pos = self.simulation.position
        txt = f"x: {pos['x']:.3f}, y: {pos['y']:.3f}, theta: {pos['theta']:.3f}"
        label = self.font.render(txt, 1, (255, 255, 255))
        rect = label.get_rect()
        rect.bottomleft = (20, self.window.get_height() - 10)
        self.window.blit(label, rect)

    def draw_path(self):
        for point in self.visited:
            self.display.set_at(point, (255, 0, 0))

    def reset(self):
        self.visited = set()
