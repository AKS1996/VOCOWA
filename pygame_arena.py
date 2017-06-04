import pygame, pygame.locals
from PIL import Image, ImageOps

BLACK = (0, 0, 0)
DGREY = (86, 86, 86)
LGREY = (172, 172, 172)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)

# in cm, using standard PC Screen Coordinates
botY = 60
botX = 60
size = [800, 600]


def main():
    pygame.init()
    screen = pygame.display.set_mode(size)
    clock = pygame.time.Clock()

    screen.fill(WHITE)
    pygame.draw.rect(screen, BLUE, [(size[0] - botX) / 2, (size[1] - botY) / 2, botX, botY])
    pygame.display.update()

    done = False
    mouse_pressed = False
    while not done:

        # This will limit the loop to 10 times per sec
        # comment this and program will use all CPU it can
        clock.tick(20)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pressed = True
            elif event.type == pygame.MOUSEBUTTONUP:
                mouse_pressed = False
            elif event.type == pygame.MOUSEMOTION and mouse_pressed:
                pygame.draw.circle(screen, BLACK, event.pos, 2)

        pygame.display.flip()  # to update screen. This must happen after all commands

    pygame.draw.rect(screen, WHITE, [(size[0] - botX) / 2, (size[1] - botY) / 2, botX, botY])
    pygame.image.save(screen, 'map.jpg')
    im = Image.open('map.jpg')
    im = im.convert('L')
    im = ImageOps.invert(im)
    im.save('map.jpg')  # saving it as  pure BW
    pygame.quit()     # being IDLE friendly


if __name__ == '__main__':
    main()
