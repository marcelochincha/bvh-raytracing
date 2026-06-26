#include <renderer/sr_ocl.hpp>

// =============================================================================
// Build with -DENABLE_OPENCL to compile the GPU offload below; without it the
// whole OpenCL backend (and the CL/cl.h dependency + -lOpenCL link) is dropped
// and we emit no-op stubs. The caller already falls back to the CPU ray tracer
// whenever ocl::available() is false, so nothing else changes.
// =============================================================================
#ifdef ENABLE_OPENCL

#define CL_TARGET_OPENCL_VERSION 120
#ifdef __APPLE__
#  include <OpenCL/cl.h>
#else
#  include <CL/cl.h>
#endif

#include <cstdio>
#include <cstddef>

namespace ocl {

// --- The kernel. Mirrors trace_ray / BVH traversal from sr_game.cpp + bvh.cpp,
// de-recursivized into a bounce loop. -------------------------------------------
static const char* KERNEL_SRC = R"CLC(
// MAXD / AMB / SEPS are injected via -D at build time from the C++ constants.
#ifndef MAXD
#define MAXD 3
#endif
#ifndef AMB
#define AMB  0.5f
#endif
#ifndef SEPS
#define SEPS 1e-4f
#endif
#define TEPS 1e-8f

inline float3 ld3(__global const float* a, int i){ return (float3)(a[i],a[i+1],a[i+2]); }

inline bool tri_hit(float3 v0,float3 v1,float3 v2,float3 o,float3 d,float* t){
    float3 e1=v1-v0, e2=v2-v0;
    float3 h=cross(d,e2);
    float a=dot(e1,h);
    if(fabs(a)<TEPS) return false;
    float f=1.0f/a;
    float3 s=o-v0;
    float u=f*dot(s,h);
    if(u<0.0f||u>1.0f) return false;
    float3 q=cross(s,e1);
    float v=f*dot(d,q);
    if(v<0.0f||u+v>1.0f) return false;
    *t=f*dot(e2,q);
    return *t>TEPS;
}

inline bool aabb_hit(float3 bmin,float3 bmax,float3 o,float3 inv,float tmax,float* tnear){
    float tmin=0.0f, tmx=tmax, t0, t1, tt;
    t0=(bmin.x-o.x)*inv.x; t1=(bmax.x-o.x)*inv.x; if(inv.x<0.0f){tt=t0;t0=t1;t1=tt;} tmin=t0>tmin?t0:tmin; tmx=t1<tmx?t1:tmx; if(tmx<tmin) return false;
    t0=(bmin.y-o.y)*inv.y; t1=(bmax.y-o.y)*inv.y; if(inv.y<0.0f){tt=t0;t0=t1;t1=tt;} tmin=t0>tmin?t0:tmin; tmx=t1<tmx?t1:tmx; if(tmx<tmin) return false;
    t0=(bmin.z-o.z)*inv.z; t1=(bmax.z-o.z)*inv.z; if(inv.z<0.0f){tt=t0;t0=t1;t1=tt;} tmin=t0>tmin?t0:tmin; tmx=t1<tmx?t1:tmx; if(tmx<tmin) return false;
    *tnear=tmin; return true;
}

// Nearest hit in one BVH. Nodes are float4 pairs (lo, hi); links are int4
// (left,right,start,count). Children bounds are tested before pushing (misses
// skip); the nearer child is pushed last so it pops first -> best shrinks sooner.
int bvh_traverse(__global const float4* nb, __global const int4* nl, int nn,
                 __global const float* tris, float3 o, float3 d, float3 inv, float* best){
    if(nn<=0) return -1;
    float tn;
    if(!aabb_hit(nb[0].xyz, nb[1].xyz, o, inv, *best, &tn)) return -1;
    int besti=-1;
    int stack[64]; int sp=0; stack[sp++]=0;
    while(sp>0){
        int ni=stack[--sp];
        int4 lk=nl[ni];
        if(lk.x<0){
            int start=lk.z, cnt=lk.w;
            for(int i=start;i<start+cnt;i++){
                float3 v0=ld3(tris,i*16), v1=ld3(tris,i*16+3), v2=ld3(tris,i*16+6);
                float t;
                if(tri_hit(v0,v1,v2,o,d,&t) && t<*best){ *best=t; besti=i; }
            }
        } else {
            int cl=lk.x, cr=lk.y;
            float tl, tr;
            bool hl=aabb_hit(nb[cl*2].xyz, nb[cl*2+1].xyz, o, inv, *best, &tl);
            bool hr=aabb_hit(nb[cr*2].xyz, nb[cr*2+1].xyz, o, inv, *best, &tr);
            if(hl && hr){
                if(sp+2<=64){
                    if(tl<tr){ stack[sp++]=cr; stack[sp++]=cl; }
                    else     { stack[sp++]=cl; stack[sp++]=cr; }
                }
            } else if(hl){ if(sp<64) stack[sp++]=cl; }
            else if(hr){ if(sp<64) stack[sp++]=cr; }
        }
    }
    return besti;
}

// Any-hit (shadow ray). Stop at the first hit.
bool bvh_occluded(__global const float4* nb, __global const int4* nl, int nn,
                  __global const float* tris, float3 o, float3 d, float3 inv, float maxt){
    if(nn<=0) return false;
    float tn;
    if(!aabb_hit(nb[0].xyz, nb[1].xyz, o, inv, maxt, &tn)) return false;
    int stack[64]; int sp=0; stack[sp++]=0;
    while(sp>0){
        int ni=stack[--sp];
        int4 lk=nl[ni];
        if(lk.x<0){
            int start=lk.z, cnt=lk.w;
            for(int i=start;i<start+cnt;i++){
                float3 v0=ld3(tris,i*16), v1=ld3(tris,i*16+3), v2=ld3(tris,i*16+6);
                float t;
                if(tri_hit(v0,v1,v2,o,d,&t) && t<maxt) return true;
            }
        } else {
            int cl=lk.x, cr=lk.y;
            float tl, tr;
            if(aabb_hit(nb[cl*2].xyz, nb[cl*2+1].xyz, o, inv, maxt, &tl)){ if(sp<64) stack[sp++]=cl; }
            if(aabb_hit(nb[cr*2].xyz, nb[cr*2+1].xyz, o, inv, maxt, &tr)){ if(sp<64) stack[sp++]=cr; }
        }
    }
    return false;
}

float3 sample_face(__global const uint* px,int off,int w,int h,float u,float v){
    if(w<=0) return (float3)(0.0f,0.0f,0.0f);
    int x=(int)(u*w); x=clamp(x,0,w-1);
    int y=(int)(v*h); y=clamp(y,0,h-1);
    uint c=px[off + y*w + x];
    return (float3)(((c>>16)&0xFF)/255.0f, ((c>>8)&0xFF)/255.0f, (c&0xFF)/255.0f);
}

// Mirror sample_sky() from sr_game.cpp so GPU/CPU skies agree.
float3 sample_sky(__global const uint* px,__global const int* offs,
                  __global const int* ws,__global const int* hs, float3 d){
    float ax=fabs(d.x), ay=fabs(d.y), az=fabs(d.z);
    float m=fmax(ax,fmax(ay,az));
    if(m<=0.0f) return (float3)(0.0f,0.0f,0.0f);
    float px_=d.x/m, py=d.y/m, pz=d.z/m;
    int idx; float u,v;
    if(ax>=ay && ax>=az){ if(px_>0.0f){idx=2;u=pz*0.5f+0.5f;v=py*0.5f+0.5f;} else {idx=1;u=0.5f-pz*0.5f;v=py*0.5f+0.5f;} }
    else if(ay>=ax && ay>=az){ if(py>0.0f){idx=4;u=px_*0.5f+0.5f;v=pz*0.5f+0.5f;} else {idx=5;u=px_*0.5f+0.5f;v=0.5f-pz*0.5f;} }
    else { if(pz>0.0f){idx=3;u=0.5f-px_*0.5f;v=py*0.5f+0.5f;} else {idx=0;u=px_*0.5f+0.5f;v=py*0.5f+0.5f;} }
    return sample_face(px,offs[idx],ws[idx],hs[idx],u,v);
}

__kernel void trace(
    // dynamic BVH (floor + pedestrians, rebuilt each frame)
    __global const float4* dnb,__global const int4* dnl,int dnn,__global const float* dtris,
    // static BVH (city / field, uploaded once)
    __global const float4* rnb,__global const int4* rnl,int rnn,__global const float* rtris,
    // skybox cubemap
    __global const uint* skypx,__global const int* skyoff,__global const int* skyw,__global const int* skyh,
    // camera
    float cpx,float cpy,float cpz,
    float axx,float axy,float axz, float ayx,float ayy,float ayz, float azx,float azy,float azz,
    float tb,float ta, float sunx,float suny,float sunz,
    int W,int H, __global uint* out)
{
    int gx=get_global_id(0), gy=get_global_id(1);
    if(gx>=W || gy>=H) return;

    float ndcX=(2.0f*(gx+0.5f))/W - 1.0f;
    float ndcY=1.0f - (2.0f*(gy+0.5f))/H;
    float3 AX=(float3)(axx,axy,axz), AY=(float3)(ayx,ayy,ayz), AZ=(float3)(azx,azy,azz);
    float3 dir=normalize(ndcX*tb*AX + ndcY*ta*AY - AZ);
    float3 orig=(float3)(cpx,cpy,cpz);
    float3 sun=(float3)(sunx,suny,sunz);

    float3 color=(float3)(0.0f,0.0f,0.0f);
    float3 tp=(float3)(1.0f,1.0f,1.0f);

    for(int depth=0; depth<=MAXD; depth++){
        float best=1e30f;
        float3 inv=(float3)(1.0f/dir.x, 1.0f/dir.y, 1.0f/dir.z);
        int di=bvh_traverse(dnb,dnl,dnn,dtris,orig,dir,inv,&best);
        int ri=bvh_traverse(rnb,rnl,rnn,rtris,orig,dir,inv,&best);

        __global const float* htris; int hidx;
        if(ri>=0){ htris=rtris; hidx=ri; }
        else if(di>=0){ htris=dtris; hidx=di; }
        else { color += tp*sample_sky(skypx,skyoff,skyw,skyh,dir); break; }

        float3 P=orig + dir*best;
        float3 N=ld3(htris,hidx*16+9);
        if(dot(N,dir)>0.0f) N=-N;
        float3 albedo=ld3(htris,hidx*16+12);
        float refl=htris[hidx*16+15];

        float ndl = fmax(0.0f, dot(N, sun));
        float sunl = 0.0f;
        if(ndl > 0.0f){
            float3 so=P+N*SEPS;
            float3 sinv=(float3)(1.0f/sun.x,1.0f/sun.y,1.0f/sun.z);
            bool insh = bvh_occluded(rnb,rnl,rnn,rtris,so,sun,sinv,1e30f)
                     || bvh_occluded(dnb,dnl,dnn,dtris,so,sun,sinv,1e30f);
            if(!insh) sunl = ndl*(1.0f-AMB);
        }

        float3 amb=sample_sky(skypx,skyoff,skyw,skyh,N)*AMB;
        float3 light=(float3)(amb.x+sunl, amb.y+sunl, amb.z+sunl);
        float3 loc=(float3)(albedo.x*light.x, albedo.y*light.y, albedo.z*light.z);

        if(depth>=MAXD || refl<=0.0f){ color += tp*loc; break; }
        color += tp*(1.0f-refl)*loc;
        tp *= refl;
        dir = dir - N*(2.0f*dot(dir,N));
        orig = P + N*SEPS;
    }

    uint r=(uint)(clamp(color.x,0.0f,1.0f)*255.0f+0.5f);
    uint g=(uint)(clamp(color.y,0.0f,1.0f)*255.0f+0.5f);
    uint b=(uint)(clamp(color.z,0.0f,1.0f)*255.0f+0.5f);
    out[gy*W+gx]=0xFF000000u | (r<<16) | (g<<8) | b;
}
)CLC";

// --- host state -------------------------------------------------------------
static bool             g_ok = false;
static cl_context       g_ctx = 0;
static cl_command_queue g_queue = 0;
static cl_program       g_prog = 0;
static cl_kernel        g_kern = 0;
static cl_device_id     g_dev = 0;

static cl_mem g_rnb=0, g_rnl=0, g_rtris=0; static int g_rnn=0;
static cl_mem g_dnb=0, g_dnl=0, g_dtris=0; static int g_dnn=0;
static cl_mem g_skypx=0, g_skyoff=0, g_skyw=0, g_skyh=0;
static cl_mem g_out=0; static int g_outw=0, g_outh=0;
static char g_devname[256] = {0};

const char* device_name(){ return g_devname; }

static cl_mem buf_ro(const void* data, size_t bytes){
    if(bytes==0){ bytes=4; data=nullptr; }
    cl_int err;
    cl_mem_flags f = CL_MEM_READ_ONLY | (data ? CL_MEM_COPY_HOST_PTR : 0);
    cl_mem m = clCreateBuffer(g_ctx, f, bytes, (void*)data, &err);
    if(err!=CL_SUCCESS){ printf("[ocl] clCreateBuffer failed (%d)\n", err); return 0; }
    return m;
}

bool available(){ return g_ok; }

bool init(int max_ray_depth, float ambient, float shadow_eps){
    cl_uint nplat=0;
    if(clGetPlatformIDs(0,nullptr,&nplat)!=CL_SUCCESS || nplat==0){
        printf("[ocl] no OpenCL platform; using CPU fallback\n"); return false;
    }
    cl_platform_id plat;
    clGetPlatformIDs(1,&plat,nullptr);

    if(clGetDeviceIDs(plat,CL_DEVICE_TYPE_GPU,1,&g_dev,nullptr)!=CL_SUCCESS){
        if(clGetDeviceIDs(plat,CL_DEVICE_TYPE_ALL,1,&g_dev,nullptr)!=CL_SUCCESS){
            printf("[ocl] no OpenCL device; using CPU fallback\n"); return false;
        }
    }

    cl_int err;
    g_ctx = clCreateContext(nullptr,1,&g_dev,nullptr,nullptr,&err);
    if(err!=CL_SUCCESS){ printf("[ocl] clCreateContext failed (%d)\n",err); return false; }
    g_queue = clCreateCommandQueue(g_ctx,g_dev,0,&err);
    if(err!=CL_SUCCESS){ printf("[ocl] clCreateCommandQueue failed (%d)\n",err); return false; }

    g_prog = clCreateProgramWithSource(g_ctx,1,&KERNEL_SRC,nullptr,&err);
    if(err!=CL_SUCCESS){ printf("[ocl] clCreateProgramWithSource failed (%d)\n",err); return false; }
    char opts[256];
    snprintf(opts, sizeof(opts),
             "-cl-fast-relaxed-math -D MAXD=%d -D AMB=%ff -D SEPS=%ff",
             max_ray_depth, ambient, shadow_eps);
    err = clBuildProgram(g_prog,1,&g_dev,opts,nullptr,nullptr);
    if(err!=CL_SUCCESS){
        char log[8192]={0}; size_t n=0;
        clGetProgramBuildInfo(g_prog,g_dev,CL_PROGRAM_BUILD_LOG,sizeof(log),log,&n);
        printf("[ocl] kernel build failed (%d):\n%s\n",err,log); return false;
    }
    g_kern = clCreateKernel(g_prog,"trace",&err);
    if(err!=CL_SUCCESS){ printf("[ocl] clCreateKernel failed (%d)\n",err); return false; }

    clGetDeviceInfo(g_dev,CL_DEVICE_NAME,sizeof(g_devname),g_devname,nullptr);
    printf("[ocl] ray tracing on GPU: %s\n", g_devname);
    g_ok = true;
    return true;
}

void set_room(const float* nb, const int* nl, const float* tris, int nnodes, int ntris){
    if(!g_ok) return;
    g_rnn = nnodes;
    g_rnb   = buf_ro(nb,   (size_t)nnodes*8*sizeof(float));
    g_rnl   = buf_ro(nl,   (size_t)nnodes*4*sizeof(int));
    g_rtris = buf_ro(tris, (size_t)ntris*16*sizeof(float));
    printf("[ocl] field BVH uploaded: %d nodes, %d tris (%s)\n",
           nnodes, ntris,
           (g_rnb && g_rnl && g_rtris) ? "OK" : "FAILED");
}

void set_sky(const uint32_t* px,int npx,const int* off,const int* w,const int* h){
    if(!g_ok) return;
    g_skypx  = buf_ro(px,  (size_t)npx*sizeof(uint32_t));
    g_skyoff = buf_ro(off, 6*sizeof(int));
    g_skyw   = buf_ro(w,   6*sizeof(int));
    g_skyh   = buf_ro(h,   6*sizeof(int));
}

static void upload_reuse(cl_mem* buf, size_t* cap, const void* data, size_t bytes){
    if(bytes==0) bytes=4;
    if(*buf==0 || bytes>*cap){
        if(*buf) clReleaseMemObject(*buf);
        cl_int err;
        *buf = clCreateBuffer(g_ctx, CL_MEM_READ_ONLY, bytes, nullptr, &err);
        *cap = bytes;
        if(err!=CL_SUCCESS){ printf("[ocl] dyn clCreateBuffer failed (%d)\n",err); *buf=0; return; }
    }
    if(data) clEnqueueWriteBuffer(g_queue, *buf, CL_FALSE, 0, bytes, data, 0, nullptr, nullptr);
}

void set_dynamic(const float* nb, const int* nl, const float* tris, int nnodes, int ntris){
    if(!g_ok) return;
    static size_t cap_nb=0, cap_nl=0, cap_tris=0;
    g_dnn = nnodes;
    upload_reuse(&g_dnb,   &cap_nb,   nb,   (size_t)nnodes*8*sizeof(float));
    upload_reuse(&g_dnl,   &cap_nl,   nl,   (size_t)nnodes*4*sizeof(int));
    upload_reuse(&g_dtris, &cap_tris, tris, (size_t)ntris*16*sizeof(float));
}

void render(float cpx,float cpy,float cpz,
            float axx,float axy,float axz,
            float ayx,float ayy,float ayz,
            float azx,float azy,float azz,
            float tb,float ta,
            float sunx,float suny,float sunz,
            int W,int H,uint32_t* out){
    if(!g_ok) return;

    if(g_out==0 || g_outw!=W || g_outh!=H){
        if(g_out) clReleaseMemObject(g_out);
        cl_int err;
        g_out = clCreateBuffer(g_ctx, CL_MEM_WRITE_ONLY, (size_t)W*H*sizeof(uint32_t), nullptr, &err);
        g_outw=W; g_outh=H;
    }

    int a=0;
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_dnb);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_dnl);
    clSetKernelArg(g_kern,a++,sizeof(int),&g_dnn);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_dtris);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_rnb);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_rnl);
    clSetKernelArg(g_kern,a++,sizeof(int),&g_rnn);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_rtris);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_skypx);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_skyoff);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_skyw);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_skyh);
    clSetKernelArg(g_kern,a++,sizeof(float),&cpx);
    clSetKernelArg(g_kern,a++,sizeof(float),&cpy);
    clSetKernelArg(g_kern,a++,sizeof(float),&cpz);
    clSetKernelArg(g_kern,a++,sizeof(float),&axx);
    clSetKernelArg(g_kern,a++,sizeof(float),&axy);
    clSetKernelArg(g_kern,a++,sizeof(float),&axz);
    clSetKernelArg(g_kern,a++,sizeof(float),&ayx);
    clSetKernelArg(g_kern,a++,sizeof(float),&ayy);
    clSetKernelArg(g_kern,a++,sizeof(float),&ayz);
    clSetKernelArg(g_kern,a++,sizeof(float),&azx);
    clSetKernelArg(g_kern,a++,sizeof(float),&azy);
    clSetKernelArg(g_kern,a++,sizeof(float),&azz);
    clSetKernelArg(g_kern,a++,sizeof(float),&tb);
    clSetKernelArg(g_kern,a++,sizeof(float),&ta);
    clSetKernelArg(g_kern,a++,sizeof(float),&sunx);
    clSetKernelArg(g_kern,a++,sizeof(float),&suny);
    clSetKernelArg(g_kern,a++,sizeof(float),&sunz);
    clSetKernelArg(g_kern,a++,sizeof(int),&W);
    clSetKernelArg(g_kern,a++,sizeof(int),&H);
    clSetKernelArg(g_kern,a++,sizeof(cl_mem),&g_out);

    // 8x8 work-groups: neighbouring pixels share BVH cache lines.
    size_t local[2]  = { 8, 8 };
    size_t global[2] = { (size_t)((W + 7) / 8 * 8), (size_t)((H + 7) / 8 * 8) };
    cl_int err = clEnqueueNDRangeKernel(g_queue,g_kern,2,nullptr,global,local,0,nullptr,nullptr);
    if(err!=CL_SUCCESS){ printf("[ocl] enqueue failed (%d); disabling GPU path\n",err); g_ok=false; return; }
    err = clEnqueueReadBuffer(g_queue,g_out,CL_TRUE,0,(size_t)W*H*sizeof(uint32_t),out,0,nullptr,nullptr);
    if(err!=CL_SUCCESS){ printf("[ocl] readback failed (%d); disabling GPU path\n",err); g_ok=false; }
}

} // namespace ocl

#else // !ENABLE_OPENCL

namespace ocl {
bool init(int, float, float) { return false; }
bool available() { return false; }
const char* device_name() { return ""; }
void set_room(const float*, const int*, const float*, int, int) {}
void set_sky(const uint32_t*, int, const int*, const int*, const int*) {}
void set_dynamic(const float*, const int*, const float*, int, int) {}
void render(float, float, float, float, float, float, float, float, float,
            float, float, float, float, float, float, float, float,
            int, int, uint32_t*) {}
} // namespace ocl

#endif // ENABLE_OPENCL
