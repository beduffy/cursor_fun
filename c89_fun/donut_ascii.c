/* C89: classic ASCII spinning donut without libm (incremental trig) */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

/* Incremental rotation helpers: rotate (cosX, sinX) by fixed (cosD, sinD) */
static void rotate(double *cosX, double *sinX, double cosD, double sinD)
{
  double c = *cosX * cosD - *sinX * sinD;
  double s = *sinX * cosD + *cosX * sinD;
  *cosX = c; *sinX = s;
}


int main(void)
{
  /* Screen settings */
  const int width = 80;
  const int height = 22;
  const double K2 = 5.0;     /* distance from viewer */
  const double R1 = 1.0;     /* donut tube radius */
  const double R2 = 2.0;     /* donut center radius */

  /* Rotation angles A, B and their per-frame step dA, dB (in radians) */
  double cosA = 1.0, sinA = 0.0;              /* A = 0 */
  const double cosdA = 0.9987502603949663;    /* cos(0.05) */
  const double sindA = 0.04997916927067833;   /* sin(0.05) */

  double cosB = 1.0, sinB = 0.0;              /* B = 0 */
  const double cosdB = 0.9995065603657316;    /* cos(0.0315) */
  const double sindB = 0.03149408018628413;   /* sin(0.0315) */

  /* Loop steps along theta (tube) and phi (around donut) */
  const double dTheta = 0.3; /* coarse for speed */
  const double dPhi = 0.07;
  const double cosdTheta = 0.955336489125606; /* cos(0.3) */
  const double sindTheta = 0.2955202066613396;/* sin(0.3) */
  const double cosdPhi = 0.9975464611571316;  /* cos(0.07) */
  const double sindPhi = 0.06994284733753277; /* sin(0.07) */

  /* Brightness ramp */
  const char *lum = " .,-~:;=!*#$@"; /* 12 levels */

  char output[80 * 22 + 1];
  double zbuffer[80 * 22];

  int frame;
  for (frame = 0; frame < 40; frame++) {
    int i;
    /* Clear buffers */
    for (i = 0; i < width * height; i++) { output[i] = ' '; zbuffer[i] = 0.0; }
    output[width * height] = '\0';

    /* For theta: initialize cos/sin = cos(0)=1, sin(0)=0 */
    {
      double cosTheta0 = 1.0, sinTheta0 = 0.0;
      double cosTheta = cosTheta0, sinTheta = sinTheta0;
      double theta;
      for (theta = 0.0; theta < 6.283185; theta += dTheta) {
        /* For phi: same incremental approach */
        double cosPhi = 1.0, sinPhi = 0.0;
        double phi;
        for (phi = 0.0; phi < 6.283185; phi += dPhi) {
          double circleX = R2 + R1 * cosTheta; /* x on torus circle */
          double circleY = R1 * sinTheta;      /* y on torus circle */

          /* 3D point after rotations around A (x-z) and then B (y-z) */
          double x = circleX * (cosB * cosPhi + sinA * sinB * sinPhi) - circleY * cosA * sinB;
          double y = circleX * (sinB * cosPhi - sinA * cosB * sinPhi) + circleY * cosA * cosB;
          double z = K2 + cosA * circleX * sinPhi + circleY * sinA;
          double invz = 1.0 / z;

          int xp = (int)(width / 2 + 30.0 * invz * x);
          int yp = (int)(height / 2 - 15.0 * invz * y);

          /* Surface normal's brightness approximation */
          {
            double nx = -cosA * cosPhi * sinTheta + sinA * cosTheta;
            double ny = -sinB * sinPhi * sinTheta - sinA * cosB * cosPhi * sinTheta + cosA * cosB * cosTheta;
            double nz = cosB * sinPhi * sinTheta - sinA * sinB * cosPhi * sinTheta + cosA * sinB * cosTheta;
            double L = nx * 0.0 + ny * 1.0 + nz * 0.0; /* light from +Y */
            int idx;

            if (xp >= 0 && xp < width && yp >= 0 && yp < height) {
              if (invz > zbuffer[xp + yp * width]) {
                zbuffer[xp + yp * width] = invz;
                idx = (int)(L * 8.0 + 4.0);
                if (idx < 0) { idx = 0; }
                if (idx > 11) { idx = 11; }
                output[xp + yp * width] = lum[idx];
              }
            }
          }

          /* advance phi */
          rotate(&cosPhi, &sinPhi, cosdPhi, sindPhi);
        }
        /* advance theta */
        rotate(&cosTheta, &sinTheta, cosdTheta, sindTheta);
      }
    }

    /* Print frame */
    {
      int y;
      printf("\x1b[H"); /* move cursor home */
      for (y = 0; y < height; y++) {
        fwrite(&output[y * width], 1u, (unsigned)width, stdout);
        putchar('\n');
      }
      fflush(stdout);
    }

    /* advance global rotations */
    rotate(&cosA, &sinA, cosdA, sindA);
    rotate(&cosB, &sinB, cosdB, sindB);

    /* Slow down: simple portable sleep using clock() busy-wait */
    {
      double ms = 50.0; /* default ~20 FPS */
      const char *env_ms = getenv("DONUT_MS");
      if (env_ms != 0) {
        int parsed = atoi(env_ms);
        if (parsed > 0) { ms = (double)parsed; }
      }
      {
        clock_t start = clock();
        clock_t wait = (clock_t)(ms * (double)CLOCKS_PER_SEC / 1000.0);
        while ((clock() - start) < wait) { /* busy-wait */ }
      }
    }
  }

  return 0;
}


