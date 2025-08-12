/* C89: minimal HTTP GET over POSIX sockets (optional; not in default run) */
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

int main(void)
{
  const char *host = "example.org"; const char *port = "80"; const char *path = "/";
  struct addrinfo hints; struct addrinfo *res=0,*rp; int s=-1; int r;
  char req[256]; char buf[512];
  memset(&hints,0,sizeof hints); hints.ai_family=AF_UNSPEC; hints.ai_socktype=SOCK_STREAM;
  if (getaddrinfo(host, port, &hints, &res)!=0){ printf("dns failed\n"); return 0; }
  for(rp=res; rp; rp=rp->ai_next){ s=socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol); if(s==-1) continue; if(connect(s, rp->ai_addr, rp->ai_addrlen)==0) break; close(s); s=-1; }
  freeaddrinfo(res); if(s==-1){ printf("connect failed\n"); return 0; }
  snprintf(req,sizeof req,"GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: c89-http\r\n\r\n", path, host);
  write(s, req, strlen(req));
  while((r=read(s, buf, sizeof buf))>0){ fwrite(buf,1u,(unsigned)r,stdout); }
  close(s);
  return 0;
}


