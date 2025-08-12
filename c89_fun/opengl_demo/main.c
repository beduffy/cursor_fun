/* Minimal GLX window (Linux, optional) */
#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <stdio.h>

int main(void)
{
  Display *d = XOpenDisplay(0);
  int screen = DefaultScreen(d);
  int attrs[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
  XVisualInfo *vi = glXChooseVisual(d, screen, attrs);
  Colormap cmap = XCreateColormap(d, RootWindow(d, vi->screen), vi->visual, AllocNone);
  XSetWindowAttributes swa;
  Window w;
  GLXContext glc;
  XEvent xev;
  int running = 1;

  swa.colormap = cmap; swa.event_mask = ExposureMask | KeyPressMask | StructureNotifyMask;
  w = XCreateWindow(d, RootWindow(d, vi->screen), 0,0, 640,480, 0, vi->depth, InputOutput, vi->visual,
                    CWColormap | CWEventMask, &swa);
  XMapWindow(d, w);
  XStoreName(d, w, "C89 GLX Demo");
  glc = glXCreateContext(d, vi, 0, GL_TRUE);
  glXMakeCurrent(d, w, glc);

  while (running) {
    while (XPending(d)) {
      XNextEvent(d, &xev);
      if (xev.type == KeyPress) running = 0;
    }
    glClearColor(0.1f, 0.2f, 0.4f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_TRIANGLES);
      glColor3f(1,0,0); glVertex2f(-0.6f,-0.6f);
      glColor3f(0,1,0); glVertex2f( 0.6f,-0.6f);
      glColor3f(0,0,1); glVertex2f( 0.0f, 0.6f);
    glEnd();
    glXSwapBuffers(d, w);
  }

  glXMakeCurrent(d, None, 0);
  glXDestroyContext(d, glc);
  XDestroyWindow(d, w);
  XCloseDisplay(d);
  return 0;
}



