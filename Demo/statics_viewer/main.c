#include "STATICS/YMPME_Statics.h"
#include "demo_host.h"
#include "YMGUI_Invalidate.h"
#include "YMGUI_Label.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { UP=1, DOWN=2, LEFT=4, RIGHT=8, ZOOM_IN=16, ZOOM_OUT=32,
       NEXT=64, RESET=128, QUIT=256 };
static const YMGRE_DemoKeyBinding keys[] = {
    {'1',0,0,1}, {'2',0,0,2}, {'3',0,0,3}, {'4',0,0,4},
    {'5',0,0,5}, {'6',0,0,6}, {'7',0,0,7}, {YMGRE_KEY_UP,0,UP,0},
    {YMGRE_KEY_DOWN,0,DOWN,0}, {YMGRE_KEY_LEFT,0,LEFT,0},
    {YMGRE_KEY_RIGHT,0,RIGHT,0}, {'=',0,ZOOM_IN,0}, {'+',0,ZOOM_IN,0},
    {'-',0,ZOOM_OUT,0}, {YMGRE_KEY_TAB,0,NEXT,0}, {'r',0,RESET,0},
    {YMGRE_KEY_ESCAPE,0,QUIT,0}
};
enum { NODE_MAX=9, ELEMENT_MAX=16, WORK_MAX=2*(3*NODE_MAX)*(3*NODE_MAX)+9*NODE_MAX };
static const char *titles[] = {"", "1/7  CANTILEVER", "2/7  TRIANGULAR TRUSS",
    "3/7  SERIES SPRINGS", "4/7  SIMPLY SUPPORTED BEAM", "5/7  CONTINUOUS BEAM",
    "6/7  PORTAL FRAME", "7/7  WARREN BRIDGE"};
typedef struct {
    pme_statics_node nodes[NODE_MAX];
    pme_statics_element elements[ELEMENT_MAX];
    pme_statics_node_result result[NODE_MAX];
    pme_statics_element_result force[ELEMENT_MAX];
    double work[WORK_MAX];
    size_t count, element_count, selected, loaded;
    int scene, bending;
    double fx, fy, magnification;
    double view_scale, view_x, view_y;
} Scene;
static void member(Scene *s, size_t a, size_t b, pme_statics_element_kind kind)
{
    pme_statics_element *e=&s->elements[s->element_count++];
    e->kind=kind; e->node_a=a; e->node_b=b;
    e->stiffness=1000;
    e->young_modulus=200e9; e->area=kind==PME_STATICS_BEAM ? .02*.04 : 1e-6;
    e->second_moment=.02*.04*.04*.04/12;
    if(kind==PME_STATICS_BEAM) s->bending=1;
}
static void reset(Scene *s, int scene)
{
    size_t i;
    memset(s,0,sizeof(*s));
    s->scene=scene; s->fy=-10; s->magnification=300;
    s->nodes[0].fixed=PME_STATICS_FIX_X|PME_STATICS_FIX_Y;
    if(scene==1) {
        s->count=2; s->loaded=1; s->nodes[1].x=2;
        s->nodes[0].fixed|=PME_STATICS_FIX_RZ;
        member(s,0,1,PME_STATICS_BEAM);
    } else if(scene==2) {
        s->count=3; s->loaded=2; s->magnification=2000;
        s->nodes[1].x=2; s->nodes[1].fixed=PME_STATICS_FIX_Y;
        s->nodes[2].x=1; s->nodes[2].y=1;
        member(s,0,1,PME_STATICS_TRUSS); member(s,0,2,PME_STATICS_TRUSS);
        member(s,1,2,PME_STATICS_TRUSS);
    } else if(scene==3) {
        s->count=3; s->loaded=2; s->fx=10; s->fy=0; s->magnification=20;
        for(i=1;i<3;++i) {
            s->nodes[i].x=(double)i; s->nodes[i].fixed=PME_STATICS_FIX_Y;
            member(s,i-1,i,PME_STATICS_SPRING);
        }
    } else if(scene==4 || scene==5) {
        s->count=scene==4 ? 3:5; s->loaded=1; s->magnification=2000;
        for(i=1;i<s->count;++i) {
            s->nodes[i].x=(double)i;
            if(i%2==0) s->nodes[i].fixed=PME_STATICS_FIX_Y;
            member(s,i-1,i,PME_STATICS_BEAM);
        }
    } else if(scene==6) {
        s->count=4; s->loaded=2; s->fx=10; s->magnification=2000;
        s->nodes[0].fixed|=PME_STATICS_FIX_RZ;
        s->nodes[1].y=1.5; s->nodes[2].x=2; s->nodes[2].y=1.5;
        s->nodes[3].x=2; s->nodes[3].fixed=7;
        member(s,0,1,PME_STATICS_BEAM); member(s,1,2,PME_STATICS_BEAM);
        member(s,2,3,PME_STATICS_BEAM);
    } else {
        s->count=9; s->loaded=2; s->magnification=1000;
        s->nodes[4].fixed=PME_STATICS_FIX_Y;
        for(i=0;i<5;++i) s->nodes[i].x=(double)i;
        for(i=0;i<4;++i) {
            s->nodes[5+i].x=i+.5; s->nodes[5+i].y=1;
            member(s,i,i+1,PME_STATICS_TRUSS);
            member(s,i,5+i,PME_STATICS_TRUSS);
            member(s,5+i,i+1,PME_STATICS_TRUSS);
            if(i<3) member(s,5+i,6+i,PME_STATICS_TRUSS);
        }
    }
    s->selected=s->loaded;
}
static int solve(Scene *s)
{
    pme_statics_model model={s->nodes,s->count,s->elements,s->element_count};
    s->nodes[s->loaded].load[0]=s->fx;
    s->nodes[s->loaded].load[1]=s->fy;
    return pme_statics_solve(&model,s->result,NODE_MAX,s->force,ELEMENT_MAX,
                            s->work,WORK_MAX)==PME_STATICS_OK;
}
/* Shared sampling keeps auto-fit and drawing consistent, including beam curvature. */
static void sample(const Scene *s,size_t index,double q,double *x,double *y,double *value)
{
    const pme_statics_element *e=&s->elements[index];
    const pme_statics_node *a=&s->nodes[e->node_a],*b=&s->nodes[e->node_b];
    const double *u=s->result[e->node_a].displacement,*v=s->result[e->node_b].displacement;
    double length=hypot(b->x-a->x,b->y-a->y),c=(b->x-a->x)/length,d=(b->y-a->y)/length;
    double q2=q*q,q3=q2*q;
    double axial=(1-q)*(c*u[0]+d*u[1])+q*(c*v[0]+d*v[1]);
    double transverse=(1-q)*(-d*u[0]+c*u[1])+q*(-d*v[0]+c*v[1]);
    *value=s->force[index].axial_force;
    if(e->kind==PME_STATICS_BEAM) {
        transverse=(1-3*q2+2*q3)*(-d*u[0]+c*u[1])+(q-2*q2+q3)*length*u[2]
                   +(3*q2-2*q3)*(-d*v[0]+c*v[1])+(q3-q2)*length*v[2];
        *value=-(1-q)*s->force[index].end_force[2]+q*s->force[index].end_force[5];
    }
    *x=a->x+q*(b->x-a->x)+s->magnification*(c*axial-d*transverse);
    *y=a->y+q*(b->y-a->y)+s->magnification*(d*axial+c*transverse);
}
static void fit(Scene *s)
{
    double xmin=0,xmax=2,ymin=-.4,ymax=1.1;
    size_t i,j;
    for(i=0;i<s->count;++i) {
        xmin=fmin(xmin,s->nodes[i].x);xmax=fmax(xmax,s->nodes[i].x);
        ymin=fmin(ymin,s->nodes[i].y);ymax=fmax(ymax,s->nodes[i].y);
    }
    for(i=0;i<s->element_count;++i) for(j=0;j<=80;++j) {
        double x,y,v;
        sample(s,i,j/80.,&x,&y,&v);
        xmin=fmin(xmin,x);xmax=fmax(xmax,x);ymin=fmin(ymin,y);ymax=fmax(ymax,y);
    }
    s->view_scale=fmin(630/(xmax-xmin),310/(ymax-ymin));
    s->view_x=405-s->view_scale*(xmin+xmax)/2;
    s->view_y=235+s->view_scale*(ymin+ymax)/2;
}
static void pixel(GRE_RenderTarget t,int x,int y,GRErgb24 c)
{
    if(x>=0 && y>=0 && x<t->width && y<t->height)
        t->data[(size_t)y*t->width+x]=GRE_FramePixel_From_RGB24(c);
}
static void line(GRE_RenderTarget t,int x,int y,int xx,int yy,GRErgb24 c,int width)
{
    int dx=abs(xx-x),sx=x<xx?1:-1,dy=-abs(yy-y),sy=y<yy?1:-1,err=dx+dy;
    for(;;) {
        int a,b,e=2*err;
        for(a=-width;a<=width;++a) for(b=-width;b<=width;++b) pixel(t,x+a,y+b,c);
        if(x==xx && y==yy) break;
        if(e>=dy) {err+=dy;x+=sx;}
        if(e<=dx) {err+=dx;y+=sy;}
    }
}
static int px(const Scene *s,double x) {return (int)lround(s->view_x+s->view_scale*x);}
static int py(const Scene *s,double y) {return (int)lround(s->view_y-s->view_scale*y);}
static const GRErgb24 grey={97,113,132}, gold={255,207,92}, green={78,223,170};
static void arrow(GRE_RenderTarget t,int x,int y,double fx,double fy,GRErgb24 c)
{
    double n=hypot(fx,fy),dx,dy,len;
    int ex,ey;
    if(n<1e-9) return;
    len=25+fmin(n,40); dx=fx/n;dy=-fy/n;
    ex=x+(int)(len*dx);ey=y+(int)(len*dy);
    line(t,x,y,ex,ey,c,1);
    line(t,ex,ey,ex-(int)(10*dx+5*dy),ey-(int)(10*dy-5*dx),c,1);
    line(t,ex,ey,ex-(int)(10*dx-5*dy),ey-(int)(10*dy+5*dx),c,1);
}
static GRErgb24 heat(double value,double maximum)
{
    double q=maximum>1e-12 ? fmin(1,fabs(value)/maximum) : 0;
    /* Fixed sign convention: red positive, blue negative, pale grey zero. */
    return value>=0 ? (GRErgb24){(uint8)(180+70*q),(uint8)(190-110*q),(uint8)(200-110*q)}
                    : (GRErgb24){(uint8)(180-110*q),(uint8)(190-45*q),(uint8)(200+50*q)};
}
static double peak_force(const Scene *s)
{
    double maximum=0;
    for(size_t i=0;i<s->element_count;++i) {
        double v=s->bending ? fmax(fabs(s->force[i].end_force[2]),fabs(s->force[i].end_force[5]))
                            : fabs(s->force[i].axial_force);
        maximum=fmax(maximum,v);
    }
    return maximum;
}
static void circle(GRE_RenderTarget t,int x,int y,int radius,GRErgb24 color)
{
    for(int i=0;i<32;++i) {
        double a=i*6.283185307179586/32,b=(i+1)*6.283185307179586/32;
        line(t,x+(int)lround(radius*cos(a)),y+(int)lround(radius*sin(a)),
             x+(int)lround(radius*cos(b)),y+(int)lround(radius*sin(b)),color,0);
    }
}
/* Positive world Z moment is CCW; screen Y points down. A fixed upper arc
 * shows direction only; magnitude remains available in the node readout. */
static void moment_arrow(GRE_RenderTarget t,int x,int y,double moment)
{
    const double pi=3.14159265358979323846,radius=30;
    double sign=moment>0 ? 1:-1;
    double start=(moment>0 ? 20:160)*pi/180;
    double sweep=sign*140*pi/180,end=start+sweep;
    int lastx=x+(int)lround(radius*cos(start));
    int lasty=y-(int)lround(radius*sin(start));
    if(fabs(moment)<1e-8) return;
    for(int i=1;i<=40;++i) {
        double a=start+sweep*i/40;
        int xx=x+(int)lround(radius*cos(a)),yy=y-(int)lround(radius*sin(a));
        line(t,lastx,lasty,xx,yy,green,1);lastx=xx;lasty=yy;
    }
    double dx=-sign*sin(end),dy=-sign*cos(end);
    line(t,lastx,lasty,lastx-(int)lround(9*dx+4*dy),lasty-(int)lround(9*dy-4*dx),green,1);
    line(t,lastx,lasty,lastx-(int)lround(9*dx-4*dy),lasty-(int)lround(9*dy+4*dx),green,1);
}
static void node_marker(const Scene *s,size_t node,GRE_RenderTarget t,int x,int y)
{
    size_t beams=0,trusses=0;
    unsigned fixed=s->nodes[node].fixed;
    int pin;
    GRErgb24 color=node==s->selected ? gold:green;
    for(size_t i=0;i<s->element_count;++i) {
        const pme_statics_element *e=&s->elements[i];
        if(e->node_a==node || e->node_b==node) {
            beams+=e->kind==PME_STATICS_BEAM;
            trusses+=e->kind==PME_STATICS_TRUSS;
        }
    }
    /* Interior continuous-beam supports do not release the beam's rotation. */
    pin=trusses>0 || (beams==1 && (fixed&3) && !(fixed&PME_STATICS_FIX_RZ));
    if(pin) {
        for(int a=-4;a<=4;++a) for(int b=-4;b<=4;++b)
            if(a*a+b*b<=16) pixel(t,x+a,y+b,(GRErgb24){20,28,40});
        circle(t,x,y,5,color);
    } else {
        line(t,x,y,x,y,color,3);
    }
    /* Selection brackets do not suggest an extra hinge. */
    if(node==s->selected) {
        line(t,x-9,y-9,x-9,y-4,gold,0);line(t,x-9,y-9,x-4,y-9,gold,0);
        line(t,x+9,y+9,x+9,y+4,gold,0);line(t,x+9,y+9,x+4,y+9,gold,0);
    }
}
/* Moving assembly follows the amplified node; the ground stays at reference y.
 * An extended horizontal track shows the available sliding direction. */
static void support(const Scene *s,size_t i,GRE_RenderTarget t,int x,int y)
{
    unsigned fixed=s->nodes[i].fixed;
    int ground_y=py(s,s->nodes[i].y);
    if(fixed&PME_STATICS_FIX_RZ) {
        if(s->scene==6) {
            line(t,x-28,y+10,x+28,y+10,grey,2);
            line(t,x,y,x,y+10,grey,1);
            for(int h=-25;h<28;h+=10) line(t,x+h,y+10,x+h-10,y+23,grey,0);
        } else {
            line(t,x-10,y-28,x-10,y+28,grey,2);
            line(t,x-10,y,x,y,grey,1);
            for(int h=-25;h<28;h+=10) line(t,x-10,y+h,x-23,y+h+10,grey,0);
        }
    } else {
        line(t,x,y,x-12,y+18,grey,1);line(t,x,y,x+12,y+18,grey,1);
        line(t,x-12,y+18,x+12,y+18,grey,1);
        if(!(fixed&PME_STATICS_FIX_X)) {
            circle(t,x-7,y+23,4,grey);circle(t,x+7,y+23,4,grey);
            /* Track is fixed in screen/world reference, independent of node motion. */
            line(t,35,ground_y+28,775,ground_y+28,grey,0);
        } else {
            line(t,x-20,ground_y+22,x+20,ground_y+22,grey,1);
            for(int h=-20;h<20;h+=8) line(t,x+h,ground_y+22,x+h-5,ground_y+28,grey,0);
        }
    }
    arrow(t,x,y,s->result[i].reaction[0],s->result[i].reaction[1],green);
    moment_arrow(t,x,y,s->result[i].reaction[2]);
}
static void draw(Scene *s,GRE_RenderTarget t)
{
    size_t i,j;
    double maximum=peak_force(s);
    GRErgb24 bg={20,28,40},grid={30,42,57};
    fit(s);
    for(i=0;i<(size_t)t->width*t->height;++i) t->data[i]=GRE_FramePixel_From_RGB24(bg);
    for(int x=12;x<t->width;x+=53) line(t,x,0,x,t->height-1,grid,0);
    for(int y=37;y<t->height;y+=53) line(t,0,y,t->width-1,y,grid,0);
    for(i=0;i<s->element_count;++i) {
        const pme_statics_element *e=&s->elements[i];
        const pme_statics_node *a=&s->nodes[e->node_a],*b=&s->nodes[e->node_b];
        int lastx=0,lasty=0;
        line(t,px(s,a->x),py(s,a->y),px(s,b->x),py(s,b->y),grey,0);
        for(j=0;j<=80;++j) {
            double xx,yy,value;
            int x,y;
            sample(s,i,j/80.,&xx,&yy,&value);
            x=px(s,xx);y=py(s,yy);
            /* Coil symbol is schematic, not transverse spring deformation. */
            if(e->kind==PME_STATICS_SPRING && j>8 && j<72)
                y+=(int)lround(7*sin((j-8)*6.283185307179586/8));
            if(j) line(t,lastx,lasty,x,y,heat(value,maximum),2);
            lastx=x;lasty=y;
        }
    }
    for(i=0;i<s->count;++i) {
        int x=px(s,s->nodes[i].x+s->magnification*s->result[i].displacement[0]);
        int y=py(s,s->nodes[i].y+s->magnification*s->result[i].displacement[1]);
        if(s->nodes[i].fixed) support(s,i,t,x,y);
        node_marker(s,i,t,x,y);
        if(i==s->loaded) arrow(t,x,y,s->fx,s->fy,gold);
    }
}
static GYOBJ label(GYOBJ parent,int x,int y,int width,const char *text)
{
    GYOBJ l=YMGUI_Creat_Label_Creat(parent,x,y,width,24);
    if(l) {YMGUI_Label_SetText(l,text);YMGUI_Label_SetTextColor(l,GY_ARGB(255,222,232,243));}
    return l;
}
static uint64_t hash(GRE_RenderTarget t)
{
    const unsigned char *p=(const unsigned char *)t->data;
    size_t i; uint64_t h=UINT64_C(14695981039346656037);
    for(i=0;i<(size_t)t->width*t->height*sizeof(*t->data);++i) h=(h^p[i])*UINT64_C(1099511628211);
    return h;
}
static int balanced(const Scene *s)
{
    double fx=0,fy=0,m=0; size_t i;
    for(i=0;i<s->count;++i) {
        double x=s->nodes[i].load[0]+s->result[i].reaction[0];
        double y=s->nodes[i].load[1]+s->result[i].reaction[1];
        fx+=x;fy+=y;m+=s->nodes[i].x*y-s->nodes[i].y*x+s->nodes[i].load[2]+s->result[i].reaction[2];
    }
    return fabs(fx)<1e-7 && fabs(fy)<1e-7 && fabs(m)<1e-7;
}
/* Analytical checks are independent of the renderer and compare unscaled results. */
static int reference_check(const Scene *s)
{
    const double tol=1e-7,ei=200e9*(.02*.04*.04*.04/12);
    if(s->scene==1) return fabs(s->result[1].displacement[1]+.00125)<1e-10;
    if(s->scene==2) return fabs(s->result[0].reaction[1]-5)<tol &&
        fabs(s->result[1].reaction[1]-5)<tol && fabs(s->result[1].displacement[0]-.00005)<1e-10;
    if(s->scene==3) return fabs(s->result[2].displacement[0]-.02)<1e-10 &&
        fabs(s->force[0].axial_force-10)<tol && fabs(s->force[1].axial_force-10)<tol;
    if(s->scene==4) return fabs(s->result[1].displacement[1]+10*8/(48*ei))<1e-10 &&
        fabs(s->result[0].reaction[1]-5)<tol && fabs(s->result[2].reaction[1]-5)<tol;
    /* Two equal spans, a central point load in the left span only. */
    if(s->scene==5) return fabs(s->result[0].reaction[1]-4.0625)<tol &&
        fabs(s->result[2].reaction[1]-6.875)<tol && fabs(s->result[4].reaction[1]+.9375)<tol;
    if(s->scene==6) return s->result[2].displacement[0]>0 &&
        fabs(s->result[0].displacement[0])+fabs(s->result[3].displacement[1])<1e-12 &&
        fabs(s->result[0].reaction[0]+s->result[3].reaction[0]+10)<tol;
    return fabs(s->result[0].reaction[1]-5)<tol && fabs(s->result[4].reaction[1]-5)<tol &&
        s->result[2].displacement[1]<0;
}
int main(int argc,char **argv)
{
    YMGRE_DemoHost host;
    GRE_RenderTarget target;
    GYOBJ image,rows[12],force_notes[3];
    Scene s;
    int smoke=0,rc=1,start_scene=1;
    if(argc==2 && !strcmp(argv[1],"--smoke-test")) smoke=1;
    else if(argc==3 && !strcmp(argv[1],"--scene") && strlen(argv[2])==1 && argv[2][0]>='1' && argv[2][0]<='7')
        start_scene=argv[2][0]-'0';
    else if(argc!=1) {fprintf(stderr,"Usage: %s [--scene 1..7 | --smoke-test]\n",argv[0]);return 1;}
    unsigned frame=0,seen=0;
    uint64_t first=0;
    unsigned covered=0;
    double initial_scale=0;
    reset(&s,start_scene);
    if(!solve(&s)) return 1;
    YMGRE_DemoHost_DefaultScale(1);
    if(!YMGRE_DemoHost_Init(&host,1160,650,40)) {
        fprintf(stderr,"GRE window: %s\n",YMGRE_DemoHost_LastError());return 1;
    }
    YMGRE_DemoHost_SetTitle(&host,"YMPME | 2D statics - seven structural examples");
    target=YMGRE_Creat_RenderTarget(810,490);
    if(!target) goto cleanup;
    image=YMGRE_DemoHost_AddTarget(&host,target,10,60,810,490);
    if(!image || !label(host.context->root,18,16,1120,"YMPME / STATICS     7 examples     Press 1-7 to switch")) goto cleanup;
    for(int i=0;i<12;++i) {
        rows[i]=label(host.context->root,835,66+38*i,315,"");
        if(!rows[i]) goto cleanup;
    }
    YMGUI_Label_SetTextColor(rows[0],GY_ARGB(255,255,207,92));
    for(int i=0;i<3;++i) {
        force_notes[i]=label(host.context->root,25,70+25*i,780,"");
        if(!force_notes[i]) goto cleanup;
    }
    /* GUI labels hold 63 characters; keep each instruction in a short column. */
    if(!label(host.context->root,18,560,550,"1 Beam  2 Truss  3 Springs  4 Simple beam") ||
       !label(host.context->root,585,560,560,"5 Continuous beam  6 Frame  7 Bridge") ||
       !label(host.context->root,18,589,550,"Arrows: load +/- 5 N   TAB: inspect node") ||
       !label(host.context->root,585,589,560,"+/-: scale   R: reset   ESC: exit") ||
       !label(host.context->root,18,618,550,"Grey: reference  Yellow: load  Green: reaction") ||
       !label(host.context->root,585,618,560,"Hollow: pin   Solid: node   M+: CCW")) goto cleanup;
    if(!YMGRE_DemoHost_BindKeys(&host,keys,sizeof keys/sizeof keys[0])) goto cleanup;
    for(;;++frame) {
        unsigned actions;
        int value;
        char text[128];
        if(smoke) {
            unsigned phase=frame%10;
            int key=frame==70 ? YMGRE_KEY_ESCAPE : phase==0 ? '1'+(int)(frame/10) :
                    phase==2 ? YMGRE_KEY_RIGHT : phase==4 ? YMGRE_KEY_TAB :
                    phase==6 ? '-' : phase==8 ? 'r' : 0;
            if(key && (!YMGRE_DemoHost_InjectKey(&host,key,1)||!YMGRE_DemoHost_InjectKey(&host,key,0))) goto cleanup;
        }
        actions=YMGRE_DemoHost_TakeActions(&host);value=YMGRE_DemoHost_TakeValue(&host);
        if(actions&QUIT) {rc=smoke && covered!=127 ? 1:0;break;}
        if(value) reset(&s,value);
        if(actions&RESET) reset(&s,s.scene);
        if(actions&UP) s.fy+=5;
        if(actions&DOWN) s.fy-=5;
        if(actions&LEFT) s.fx-=5;
        if(actions&RIGHT) s.fx+=5;
        /* Bound physical loads; magnification affects only visualization. */
        s.fx=fmax(-40,fmin(40,s.fx));s.fy=fmax(-40,fmin(40,s.fy));
        if(actions&ZOOM_IN) s.magnification*=1.25;
        if(actions&ZOOM_OUT) s.magnification/=1.25;
        s.magnification=fmax(1,fmin(10000,s.magnification));
        if(actions&NEXT) s.selected=(s.selected+1)%s.count;
        if(!solve(&s)) goto cleanup;
        draw(&s,target);
        if(s.scene==2) {
            YMGUI_Label_SetText(force_notes[0],"Arrows: loads / support reactions. Colors: member axial force.");
            snprintf(text,sizeof text,"Bottom N=%+.2f N; left N=%+.2f N; right N=%+.2f N",
                s.force[0].axial_force,s.force[1].axial_force,s.force[2].axial_force);
            YMGUI_Label_SetText(force_notes[1],text);
            YMGUI_Label_SetText(force_notes[2],"N>0: tension; N<0: compression. Rx=0 does not imply N=0.");
        } else {
            for(int i=0;i<3;++i) YMGUI_Label_SetText(force_notes[i],"");
        }
        YMGUI_Label_SetText(rows[0],titles[s.scene]);
        snprintf(text,sizeof text,"Load: Fx=%+.0f Fy=%+.0f N",s.fx,s.fy);YMGUI_Label_SetText(rows[1],text);
        snprintf(text,sizeof text,"Deformation: x%.0f",s.magnification);YMGUI_Label_SetText(rows[2],text);
        snprintf(text,sizeof text,"Node %zu / load at %zu",s.selected,s.loaded);YMGUI_Label_SetText(rows[3],text);
        snprintf(text,sizeof text,"ux = %+.5f mm",s.result[s.selected].displacement[0]*1000);YMGUI_Label_SetText(rows[4],text);
        snprintf(text,sizeof text,"uy = %+.5f mm",s.result[s.selected].displacement[1]*1000);YMGUI_Label_SetText(rows[5],text);
        snprintf(text,sizeof text,"rz = %+.6f rad",s.result[s.selected].displacement[2]);YMGUI_Label_SetText(rows[6],text);
        snprintf(text,sizeof text,"Rx=%+.2f Ry=%+.2f N",s.result[s.selected].reaction[0],s.result[s.selected].reaction[1]);YMGUI_Label_SetText(rows[7],text);
        snprintf(text,sizeof text,"Reaction Mz=%+.2f N m",s.result[s.selected].reaction[2]);YMGUI_Label_SetText(rows[8],text);
        YMGUI_Label_SetText(rows[9],s.bending?"Color: bending moment M":"Color: axial force N");
        YMGUI_Label_SetText(rows[10],s.scene==5 ? "Bilateral supports / hold-down" :
            s.scene==3 ? "Y guided / LEFT-RIGHT: stretch" :
            s.scene==2 ? "Left pin / right roller: X free" :
            s.scene==6 ? "Rigid joints / bases fixed" : "Red: positive / Blue: negative");
        double peak=peak_force(&s);
        snprintf(text,sizeof text,"Color range: +/- %.2f %s",peak,s.bending?"N m":"N");YMGUI_Label_SetText(rows[11],text);
        if(smoke) {
            unsigned phase=frame%10;
            if(!balanced(&s)) goto cleanup;
            if(phase==1) {
                if(s.scene!=(int)(frame/10)+1 || !reference_check(&s)) goto cleanup;
                first=hash(target);initial_scale=s.magnification;seen=1;
            }
            if(phase==3 && s.fx==(s.scene==3 || s.scene==6 ? 15:5) && hash(target)!=first) seen|=2;
            if(phase==5 && s.selected==(s.loaded+1)%s.count) seen|=4;
            if(phase==7 && fabs(s.magnification-initial_scale*.8)<1e-8) seen|=8;
            if(phase==9) {
                if(seen!=15 || !reference_check(&s) || s.selected!=s.loaded ||
                   s.magnification!=initial_scale) goto cleanup;
                covered|=1u<<(s.scene-1);
            }
            if(frame>75) goto cleanup;
        }
        YMGUI_Obj_Invalidate(image);
        if(!YMGRE_DemoHost_Step(&host,smoke?0:30)) {rc=smoke?1:0;break;}
    }
cleanup:
    YMGRE_DemoHost_Destroy(&host);
    if(target) YMGRE_Free_RenderTarget(target);
    if(smoke) printf("%s statics viewer: numerical balance, rendering, load, 7 scenes, selection, scale, reset (%u)\n",rc?"FAIL":"PASS",covered);
    return rc;
}
