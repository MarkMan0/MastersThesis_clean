from pygame.sprite import Sprite
import pygame
import math


class Car(Sprite):

    def __init__(self, screen):
        super().__init__()

        self.screen = screen
        self.sz = 15
        self.orig_image = pygame.Surface((self.sz,)*2, pygame.SRCALPHA)
        pygame.draw.circle(self.orig_image, (255, 0, 0),
                           (int(self.sz / 2), ) * 2, self.sz / 2)
        pygame.draw.circle(self.orig_image, (0, 0, 255),
                           (int(self.sz / 3 * 2), int(self.sz / 2)), 3)

        self.image = self.orig_image.copy()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def set_pos(self, pos):
        self.x = pos['x']
        self.y = pos['y']
        self.theta = pos['theta']

        scale = 60
        self.center = (int(self.x*scale + self.screen.get_width()/2),
                       int(self.y*scale + self.screen.get_height()/2))

    def update(self, *args):
        pass

    def render(self) -> None:
        self.image = pygame.transform.rotate(
            self.orig_image, math.degrees(-self.theta))
        rect = self.image.get_rect()
        rect.center = self.center
        self.screen.blit(self.image, rect)
