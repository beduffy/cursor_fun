import pygame
import random

pygame.init()

# Screen dimensions
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("2D Grand Theft Auto")

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
blue = (0, 0, 255)

# Player settings
player_size = 50
player_pos = [screen_width // 2, screen_height // 2]
player_speed = 5

# Car settings
car_size = 60
car_pos = [random.randint(0, screen_width - car_size), random.randint(0, screen_height - car_size)]
car_speed = 7
in_car = False

# Bullet settings
bullet_size = 5
bullet_speed = 10
bullets = []

# Game loop
game_over = False
clock = pygame.time.Clock()

def draw_player(screen, player_pos):
    pygame.draw.rect(screen, blue, (player_pos[0], player_pos[1], player_size, player_size))


def draw_car(screen, car_pos):
    pygame.draw.rect(screen, red, (car_pos[0], car_pos[1], car_size, car_size))


def draw_bullets(screen, bullets):
    for bullet in bullets:
        pygame.draw.rect(screen, black, (bullet[0], bullet[1], bullet_size, bullet_size))


while not game_over:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            game_over = True

    keys = pygame.key.get_pressed()

    if keys[pygame.K_q]:
        game_over = True

    if not in_car:
        if keys[pygame.K_LEFT]:
            player_pos[0] -= player_speed
        if keys[pygame.K_RIGHT]:
            player_pos[0] += player_speed
        if keys[pygame.K_UP]:
            player_pos[1] -= player_speed
        if keys[pygame.K_DOWN]:
            player_pos[1] += player_speed

        # Check if player enters the car
        if (car_pos[0] < player_pos[0] < car_pos[0] + car_size or car_pos[0] < player_pos[0] + player_size < car_pos[0] + car_size) and \
           (car_pos[1] < player_pos[1] < car_pos[1] + car_size or car_pos[1] < player_pos[1] + player_size < car_pos[1] + car_size):
            if keys[pygame.K_e]:  # Press 'E' to enter the car
                in_car = True
                player_speed = car_speed

    else:
        if keys[pygame.K_LEFT]:
            car_pos[0] -= car_speed
        if keys[pygame.K_RIGHT]:
            car_pos[0] += car_speed
        if keys[pygame.K_UP]:
            car_pos[1] -= car_speed
        if keys[pygame.K_DOWN]:
            car_pos[1] += car_speed
        

    if keys[pygame.K_SPACE]:
        bullet_pos = [player_pos[0] + player_size // 2, player_pos[1]]
        bullets.append(bullet_pos)

    screen.fill(white)

    if not in_car:
        draw_player(screen, player_pos)
    draw_car(screen, car_pos)
    draw_bullets(screen, bullets)

    for bullet in bullets:
        bullet[1] -= bullet_speed
        if bullet[1] < 0:
            bullets.remove(bullet)

    pygame.display.update()
    clock.tick(30)

pygame.quit()
