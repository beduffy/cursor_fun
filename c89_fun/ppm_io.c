/* C89: write and read a simple binary PPM (P6) image */
#include <stdio.h>
#include <stdlib.h>

static int write_ppm(const char *path, int w, int h)
{
  FILE *fp = fopen(path, "wb");
  int x, y;
  if (!fp) {
    printf("open failed: %s\n", path);
    return 0;
  }
  fprintf(fp, "P6\n%d %d\n255\n", w, h);
  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      unsigned char rgb[3];
      rgb[0] = (unsigned char)(x * 255 / (w - 1));
      rgb[1] = (unsigned char)(y * 255 / (h - 1));
      rgb[2] = (unsigned char)128;
      fwrite(rgb, 1u, 3u, fp);
    }
  }
  fclose(fp);
  return 1;
}


static int read_ppm_header(const char *path, int *out_w, int *out_h)
{
  FILE *fp = fopen(path, "rb");
  char header[3];
  int w = 0, h = 0, maxv = 0;
  if (!fp) return 0;
  if (fscanf(fp, "%2s", header) != 1) { fclose(fp); return 0; }
  if (header[0] != 'P' || header[1] != '6') { fclose(fp); return 0; }
  if (fscanf(fp, "%d %d %d", &w, &h, &maxv) != 3) { fclose(fp); return 0; }
  if (maxv != 255) { fclose(fp); return 0; }
  *out_w = w; *out_h = h;
  fclose(fp);
  return 1;
}


int main(void)
{
  const char *path = "/tmp/c89_image.ppm";
  int w = 64, h = 48;
  int rw = 0, rh = 0;
  if (!write_ppm(path, w, h)) {
    printf("failed to write ppm\n");
    return 0;
  }
  if (read_ppm_header(path, &rw, &rh)) {
    printf("ppm: %dx%d written to %s\n", rw, rh, path);
  } else {
    printf("failed to read header\n");
  }
  return 0;
}



