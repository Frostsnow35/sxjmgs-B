"""可复现生成问题四论文插图（参数化示意，不含正式测试数据）。"""
from __future__ import annotations

import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon, FancyArrowPatch, Rectangle
from matplotlib.lines import Line2D

OUT = Path(__file__).resolve().parent / "figures"
OUT.mkdir(exist_ok=True)
plt.rcParams.update({"font.family": "Microsoft YaHei", "axes.unicode_minus": False, "font.size": 10})
BLUE, NAVY, RED, GREY, LIGHT = "#1f4e79", "#17324d", "#b22222", "#666666", "#d9e7f0"

def save(fig, name: str) -> None:
    fig.tight_layout()
    for ext in ("png", "pdf", "svg"):
        fig.savefig(OUT / f"{name}.{ext}", dpi=320, bbox_inches="tight")
    plt.close(fig)

def fig9() -> None:
    fig, ax = plt.subplots(figsize=(7.2, 5.3))
    g=np.array([0.,0.]); phi=28.; r=3.
    ax.add_patch(Circle(g,r,fc=LIGHT,ec=BLUE,lw=1.5,alpha=.35))
    ax.add_patch(Wedge(g,r,phi-90,phi+90,fc="#91c4df",ec="none",alpha=.55))
    u=np.array([math.cos(math.radians(phi)),math.sin(math.radians(phi))])
    ax.arrow(0,0,2.0*u[0],2.0*u[1],color=RED,width=.018,head_width=.12,length_includes_head=True)
    sa=np.array([1.45,.75]); sb=np.array([-1.7,-.9])
    for s,col in [(sa,BLUE),(sb,GREY)]:
        ax.plot(*s,"o",color=col,ms=6); ax.plot([0,s[0]],[0,s[1]],ls="--",color=col,lw=.8)
    ax.text(.12,.15,"$G_j$",color=NAVY,fontsize=12)
    ax.text(2.35,2.45,r"发射方向 $\varphi_j$",color=RED,ha="center",bbox={"fc":"white","ec":"none","pad":1.2})
    ax.text(2.45,.75,r"① 接收点 $S_a$",color=BLUE,ha="center",bbox={"fc":"white","ec":"none","pad":1.2})
    ax.text(-2.25,-1.95,r"② 背向点 $S_b$",color=GREY,ha="center",bbox={"fc":"white","ec":"none","pad":1.2})
    ax.text(0,-3.25,"接收判定：距离约束与方向约束同时满足；背向点的 no_signal 不能排除目标位置",ha="center",color=NAVY,fontsize=9)
    ax.set(xlim=(-3.5,3.5),ylim=(-3.5,3.0),aspect="equal",xlabel="$x$ / 归一化距离",ylabel="$y$ / 归一化距离")
    ax.grid(alpha=.15); save(fig,"图9_定向源距离方向联合接收")

def fig10() -> None:
    fig, ax = plt.subplots(figsize=(7.2, 6.0)); t = np.linspace(0,2*np.pi,400)
    ax.plot(1800*np.cos(t),1800*np.sin(t),color=NAVY,lw=1.4)
    ang=np.arange(7)*2*np.pi/7; pts=np.c_[1000*np.cos(ang),1000*np.sin(ang)]
    route=np.vstack([pts,pts[0]])
    ax.plot(route[:,0],route[:,1],color=BLUE,lw=1.5,marker="o",ms=5,label="正七边形主搜索")
    ax.plot(0,0,"o",color=RED,ms=6); ax.text(45,45,"原点 $S_0$",color=RED)
    # triangular lattice / perimeter probes (schematic but metric)
    tri=np.array([[-1300,-450],[-1000,70],[-700,590],[-400,-970],[350,-1050],[850,-620],[1200,-130],[1400,500]])
    ax.scatter(tri[:,0],tri[:,1],marker="^",s=38,color="#c4863f",label="三角格补充点")
    circ=np.c_[1800*np.cos(np.arange(0,2*np.pi,.55)),1800*np.sin(np.arange(0,2*np.pi,.55))]
    ax.scatter(circ[:,0],circ[:,1],s=18,color="#5c4b8a",label="圆周候选点")
    cap=np.array([[1850,0],[-1850,0],[0,1850],[0,-1850]])
    ax.scatter(cap[:,0],cap[:,1],marker="*",s=75,color=RED,label="外侧帽点")
    ax.set(xlim=(-2050,2050),ylim=(-2050,2050),aspect="equal",xlabel="$x$ / m",ylabel="$y$ / m")
    ax.legend(loc="lower left",fontsize=8,frameon=True); ax.grid(alpha=.15)
    save(fig,"图10_主搜索与补充扫描布局")

def fig11() -> None:
    fig, ax=plt.subplots(figsize=(13,3.2)); ax.axis("off")
    labels=["正七边形主搜索","方向覆盖度评价","前置补充扫描","单示向目标\n左右对称二测","可行域更新\n最小包围圆","滚动路线规划\n执行下一动作","后置保险扫描\n满足条件后结束"]
    xs=np.linspace(.07,.93,len(labels)); colors=[BLUE,BLUE,"#8b6a2f","#5c4b8a",BLUE,RED,GREY]
    for i,(x,lab,col) in enumerate(zip(xs,labels,colors)):
        ax.add_patch(Rectangle((x-.06,.36),.12,.30,fc="white",ec=col,lw=1.6))
        ax.text(x,.51,lab,ha="center",va="center",fontsize=9,color=NAVY)
        if i<len(labels)-1:
            ax.add_patch(FancyArrowPatch((x+.065,.51),(xs[i+1]-.065,.51),arrowstyle="-|>",mutation_scale=12,color=NAVY,lw=1.2))
    ax.text(.5,.10,"每完成一次检测或清除即更新状态并重新规划",ha="center",color=RED,fontsize=10)
    save(fig,"图11_问题四一体化滚动决策流程")

def fig12() -> None:
    fig,ax=plt.subplots(figsize=(8.4,5.2)); s1=np.array([0.,0.]); th=18.; u=np.array([math.cos(math.radians(th)),math.sin(math.radians(th))]); n=np.array([-u[1],u[0]])
    ax.arrow(0,0,820*u[0],820*u[1],color=BLUE,width=2,head_width=35,length_includes_head=True)
    ax.text(360, 500, "首次示向中心线 $u$", color=BLUE, ha="center", bbox={"fc":"white","ec":"none","pad":1.5})
    ax.plot(*s1,"o",color=NAVY,ms=7); ax.text(-65,-85,"$S_1$",color=NAVY)
    for sign,col,label in [(1,"#5c4b8a",r"$S_2^+$"),(-1,"#8b6a2f",r"$S_2^-$")]:
        p=120*u+sign*450*n; ax.plot(*p,"o",color=col,ms=7)
        text_offset = np.array([24, 28]) if sign == 1 else np.array([24, -38])
        ax.text(*(p+text_offset),label,color=col,
                bbox={"fc":"white","ec":"none","pad":1.2})
        ax.plot([0,p[0]],[0,p[1]],ls="--",color=col,lw=.9)
    # target uncertainty corridor
    q=np.array([680.,260.]); ax.add_patch(Polygon([q-30*n,q+30*n,q+30*n+210*u,q-30*n+210*u],fc=LIGHT,ec="none",alpha=.6))
    ax.plot(*q,"o",color=RED,ms=6); ax.text(700, 410, "首次测向得到的\n目标可能域",color=RED, ha="center",
                                             bbox={"fc":"white","ec":"none","pad":1.2})
    ax.annotate(r"沿 $u$ 前进"+"\n"+r"$120\,\mathrm{m}$",xy=120*u,xytext=(260,350),arrowprops={"arrowstyle":"->","color":BLUE},color=BLUE,ha="center")
    ax.annotate(r"法向对称偏移"+"\n"+r"$450\,\mathrm{m}$",xy=120*u+450*n,xytext=(-270,570),arrowprops={"arrowstyle":"->","color":"#5c4b8a"},color="#5c4b8a",ha="center")
    ax.set(xlim=(-550,1000),ylim=(-420,760),aspect="equal",xlabel="$x$ / m",ylabel="$y$ / m"); ax.grid(alpha=.15)
    save(fig,"图12_鲁棒二次测向点几何构造")

def fig13() -> None:
    fig=plt.figure(figsize=(10.2,5.0)); gs=fig.add_gridspec(1,2,width_ratios=[1.0,1.1],wspace=.28)
    ax1=fig.add_subplot(gs[0,0]); ax2=fig.add_subplot(gs[0,1],projection="polar")
    g=np.array([0.,0.]); sensors=np.array([[1000.,0.],[-500.,866.]])
    ax1.add_patch(Circle(g,120,fc="#f7d9d9",ec=RED,lw=1.2)); ax1.plot(*g,"o",color=RED,ms=6); ax1.text(35,35,"假设目标 $G$",color=RED)
    for i,s in enumerate(sensors,1):
        ax1.plot(*s,"o",color=NAVY,ms=6); ax1.text(*(s+np.array([35,35])),f"$S_{i}$",color=NAVY)
        ax1.plot([g[0],s[0]],[g[1],s[1]],ls="--",color=BLUE,lw=.9)
    ax1.plot(-500,-866,marker="*",color=RED,ms=11); ax1.text(-445,-900,"候选补盲点",color=RED)
    ax1.add_patch(Circle(g,1800,fill=False,ec=GREY,lw=1.0,ls=":")); ax1.set_xlim(-1250,1250); ax1.set_ylim(-1250,1250); ax1.set_aspect("equal")
    ax1.set_xlabel("空间位置 $x$ / m"); ax1.set_ylabel("空间位置 $y$ / m"); ax1.set_title("(a) 检测点与假设目标",fontsize=10); ax1.grid(alpha=.15)
    angles=np.mod(np.arctan2(sensors[:,1]-g[1],sensors[:,0]-g[0]),2*np.pi)
    theta=np.linspace(0,2*np.pi,720)
    ax2.set_theta_zero_location("E"); ax2.set_theta_direction(1); ax2.set_ylim(0,1.2); ax2.set_yticks([]); ax2.set_title("(b) 发射方向覆盖区间",pad=18,fontsize=10)
    cols=[BLUE,"#8b6a2f","#5c4b8a"]
    for a,c in zip(angles,cols):
        ax2.bar(a,1.0,width=np.pi,color=c,alpha=.28,edgecolor=c,lw=1.0)
    # Union of the semicircles, rendered as fine polar sectors.  Using small
    # bars avoids the straight-line artefact produced by fill_between at the
    # 0/360-degree seam.
    covered=np.zeros(720,dtype=bool)
    for a in angles:
        d=np.angle(np.exp(1j*(theta-a))); covered |= np.abs(d)<=np.pi/2
    dtheta=2*np.pi/720
    ax2.bar(theta[covered], 1.0, width=dtheta*1.04, bottom=0,
            color=BLUE, alpha=.45, edgecolor="none", align="center")
    ax2.text(np.deg2rad(90),1.08,"蓝色：至少一个检测点可接收",ha="center",color=BLUE,fontsize=8)
    ax2.text(np.deg2rad(270),1.08,"白色：方向盲区",ha="center",color=GREY,fontsize=8)
    fig.suptitle("方向覆盖度的几何定义（参数化示例）",fontsize=12)
    fig.text(.5,.01,r"每个检测点覆盖以其方位角为中心的 $180^\circ$ 区间；覆盖度为蓝色区间并集长度与 $360^\circ$ 之比",ha="center",fontsize=8,color=GREY)
    fig.tight_layout(rect=(0,.04,1,.95))
    for ext in ("png","pdf","svg"): fig.savefig(OUT/f"图13_方向覆盖度评价热力图.{ext}",dpi=320,bbox_inches="tight")
    plt.close(fig)
    save(fig,"图13_方向覆盖度评价热力图")

if __name__=="__main__":
    fig9(); fig10(); fig11(); fig12(); fig13(); print(f"generated in {OUT}")
