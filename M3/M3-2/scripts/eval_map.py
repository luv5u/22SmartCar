#!/usr/bin/env python3
"""Reproducible evaluation for the supplied M3-2 ring scene.
No ROS or Gazebo input; reads the SAVED map and optional odom route only.
Default ROIs assume this run starts at south-corridor midpoint, facing +x.
ROIs select measurements, never modify pixels or correct the map.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml
from scipy import ndimage as ndi
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def load_map(path):
    meta=yaml.safe_load(path.read_text())
    resolution=float(meta['resolution'])
    if resolution<=0:raise ValueError('Map resolution must be positive')
    if meta.get('mode','trinary')!='trinary':raise ValueError('This evaluator expects a trinary saved map')
    origin=np.asarray(meta['origin'],dtype=float)
    if len(origin)!=3 or not np.isfinite(origin).all():raise ValueError('Invalid map origin')
    # Our ROIs and plots are axis-aligned in map coordinates.
    if abs(origin[2])>1e-8:raise ValueError('Nonzero origin yaw: supply a coordinate-aware evaluator')
    image=Image.open(path.parent / meta['image'])
    a=np.flipud(np.asarray(image.convert('L'))).copy()
    probability=(255.-a)/255.
    if int(meta.get('negate',0)):probability=1-probability
    occupied=probability>float(meta['occupied_thresh'])
    free=probability<float(meta['free_thresh'])
    unknown=~(occupied|free)
    xx=origin[0]+(np.arange(a.shape[1])+.5)*resolution
    yy=origin[1]+(np.arange(a.shape[0])+.5)*resolution
    return meta,a,occupied,free,unknown,xx,yy


def roi_mask(xx,yy,roi):
    xmin,xmax,ymin,ymax=roi
    return ((xx[None,:]>=xmin)&(xx[None,:]<=xmax)&
            (yy[:,None]>=ymin)&(yy[:,None]<=ymax))


def fit_wall(occupied,xx,yy,roi):
    iy,ix=np.where(occupied&roi_mask(xx,yy,roi))
    if len(ix)<20:raise ValueError(f'Not enough occupied pixels in wall ROI {roi}')
    points=np.c_[xx[ix],yy[iy]]
    center=points.mean(axis=0)
    _,_,vh=np.linalg.svd(points-center,full_matrices=False)
    tangent=vh[0]
    if tangent[0]<0:tangent=-tangent
    normal=np.array([-tangent[1],tangent[0]])
    residual=(points-center)@normal
    along=(points-center)@tangent
    return dict(points=points,center=center,tangent=tangent,normal=normal,
                residual=residual,along=along,
                metrics=dict(points=len(points),rms_m=float(np.sqrt(np.mean(residual**2))),
                max_abs_m=float(np.max(np.abs(residual))),
                band90_m=float(np.quantile(residual,.95)-np.quantile(residual,.05)),
                angle_deg=float(np.degrees(np.arctan2(tangent[1],tangent[0])))))


def bands_in_wall(wall,resolution):
    # A final-map duplication screen, NOT a two-pass registration measurement.
    # In each 0.25m longitudinal slice, seek two occupied bands with an empty
    # normal bin between. A band requires >=2 occupied pixels for support.
    along,residual=wall['along'],wall['residual']
    breaks=np.arange(along.min(),along.max()+.251,.25)
    normal_breaks=np.arange(math.floor(residual.min()/resolution)*resolution-resolution,
                            residual.max()+2*resolution,resolution)
    tested=flagged=0;separations=[]
    for lo,hi in zip(breaks[:-1],breaks[1:]):
        values=residual[(along>=lo)&(along<hi)]
        if len(values)<3:continue
        counts,edges=np.histogram(values,bins=normal_breaks)
        groups,n=ndi.label(counts>0)
        centers=[]
        for label in range(1,n+1):
            ids=np.flatnonzero(groups==label)
            if counts[ids].sum()>=2:
                centers.append(float(np.average((edges[:-1][ids]+edges[1:][ids])/2,weights=counts[ids])))
        tested+=1
        if len(centers)>1:
            gaps=[b-a for a,b in zip(centers[:-1],centers[1:]) if b-a>=2*resolution-1e-9]
            if gaps:flagged+=1;separations.extend(gaps)
    return dict(test_slices=tested,multiple_band_slices=flagged,
                fraction=flagged/tested if tested else None,
                largest_band_separation_m=max(separations) if separations else None,
                definition='Final occupied bands: >=2 pixels per band; >=1 empty normal bin; centers separated >=2 cells. Not proof of loop-closure activation.')


def gate_measurement(occupied,free,xx,yy,resolution,roi):
    mask=roi_mask(xx,yy,roi)
    # Locate the vertical gate arms by occupied-pixel support in the whole ROI.
    # This avoids assuming the mapped gate is exactly at nominal x=0.
    score=(occupied&mask).sum(axis=0)
    peak=int(score.max())
    if peak<6:raise ValueError('Gate arms not found in the configured ROI')
    columns=np.flatnonzero(score>=.7*peak)
    peak_col=int(score.argmax());groups,n=ndi.label(np.isin(np.arange(len(xx)),columns))
    columns=np.flatnonzero(groups==groups[peak_col])
    rows=np.flatnonzero((yy>=roi[2])&(yy<=roi[3]))
    openings=[]
    for col in columns:
        labels,num=ndi.label(free[rows,col]); candidates=[]
        for label in range(1,num+1):
            rr=rows[labels==label]
            # Require an occupied gate arm immediately below AND above.
            if rr[0]>0 and rr[-1]<len(yy)-1 and occupied[rr[0]-1,col] and occupied[rr[-1]+1,col]:
                candidates.append(rr)
        if not candidates:continue
        rr=max(candidates,key=len)
        openings.append(dict(x_m=float(xx[col]),y_low_m=float(yy[rr[0]]-resolution/2),
                             y_high_m=float(yy[rr[-1]]+resolution/2),cells=len(rr),width_m=len(rr)*resolution))
    if not openings:raise ValueError('No free gate opening bounded by occupied arms')
    return dict(x_m=float(np.median([p['x_m'] for p in openings])),
                y_m=float(np.median([(p['y_low_m']+p['y_high_m'])/2 for p in openings])),
                width_min_m=float(min(p['width_m'] for p in openings)),
                width_max_m=float(max(p['width_m'] for p in openings)),
                min_cells=int(min(p['cells'] for p in openings)),sections=openings)


def route_metrics(path):
    with path.open() as f:rows=list(csv.DictReader(f))
    if len(rows)<2:raise ValueError('Route file has fewer than two samples')
    keys=['sim_time_s','local_x_m','local_y_m','local_yaw_rad','path_index','cross_track_m','front_range_m']
    values=np.array([[float(row[k]) for k in keys] for row in rows])
    if not np.isfinite(values[:,:6]).all():raise ValueError('Nonfinite route values')
    if np.isnan(values[:,6]).any() or np.isneginf(values[:,6]).any():raise ValueError('Invalid front ranges')
    if np.any(np.diff(values[:,0])<0):raise ValueError('Route simulation time goes backwards')
    if np.any(np.diff(values[:,4])<0):raise ValueError('Path index goes backwards')
    xy=values[:,1:3]
    return dict(rows=len(rows),duration_s=float(values[-1,0]-values[0,0]),
                max_tracking_error_m=float(np.max(values[:,5])),
                last_logged_distance_to_start_m=float(np.linalg.norm(xy[-1]-xy[0])),
                last_logged_yaw_change_deg=float(np.degrees(math.atan2(math.sin(values[-1,3]-values[0,3]),math.cos(values[-1,3]-values[0,3])))),
                min_front_range_m=float(np.min(values[:,6])),
                last_path_index=int(values[-1,4]),
                x_extent_m=[float(xy[:,0].min()),float(xy[:,0].max())],
                y_extent_m=[float(xy[:,1].min()),float(xy[:,1].max())],
                limitation='Odom frame only. Last CSV sample precedes stopping; not actual final closure error or map-frame truth.'),xy


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--map',type=Path,required=True,help='Saved map.yaml')
    ap.add_argument('--route',type=Path)
    ap.add_argument('--graph',type=Path,help='Full graph_visualization YAML; requires eval_pose_graph.py beside this script')
    ap.add_argument('--out',type=Path,default=Path('results/evaluation_02'))
    ap.add_argument('--robot-radius',type=float,default=.30,help='Conservative circular planning radius in meters; not a Nav2 footprint setting')
    ap.add_argument('--regions',type=Path,help='Optional JSON overriding scene-specific ROIs')
    args=ap.parse_args()
    if args.robot_radius<=0:raise ValueError('Robot radius must be positive')
    regions={'south_outer_wall':[-4.5,4.5,-1.2,-.5],
             'south_inner_wall':[-3.5,3.5,.6,1.4],
             'gate':[-2.,2.,7.0,9.0],
             'anchors':{'south':[-4,4,-.6,.6],'east':[5.2,7.3,2,6],
                        'north_left':[-4,-1,7.3,8.7],'north_right':[2,4,7.3,8.7],
                        'west':[-7.2,-5.2,2,6]}}
    if args.regions:regions=json.loads(args.regions.read_text())
    meta,a,occupied,free,unknown,xx,yy=load_map(args.map)
    r=float(meta['resolution']);out=args.out;out.mkdir(parents=True,exist_ok=True)
    walls={name:fit_wall(occupied,xx,yy,regions[name]) for name in ['south_outer_wall','south_inner_wall']}
    bands=bands_in_wall(walls['south_outer_wall'],r)
    gate=gate_measurement(occupied,free,xx,yy,r,regions['gate'])
    # Unknown AND occupied cells are blocked. Pad the image so its edge is blocked.
    clearance=ndi.distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*r
    clearance=np.maximum(0,clearance-r/math.sqrt(2))  # distance to cell area, conservatively
    safe=free&(clearance>=args.robot_radius)
    labels,_=ndi.label(safe)  # 4-connectivity; do not pass diagonally between obstacles
    anchors={}
    for name,roi in regions['anchors'].items():
        select=roi_mask(xx,yy,roi)
        if not np.any(select):raise ValueError(f'Empty anchor ROI {name}')
        flat=int(np.argmax(np.where(select,clearance,-1)))
        iy,ix=np.unravel_index(flat,a.shape)
        anchors[name]=dict(x_m=float(xx[ix]),y_m=float(yy[iy]),clearance_m=float(clearance[iy,ix]),label=int(labels[iy,ix]))
    anchor_labels=[p['label'] for p in anchors.values()]
    all_connected=all(label>0 and label==anchor_labels[0] for label in anchor_labels)
    # Local gate test excludes the alternate path around the rest of the ring.
    gate_crop=roi_mask(xx,yy,[gate['x_m']-.7,gate['x_m']+.7,gate['y_m']-.7,gate['y_m']+.7])
    local_labels,_=ndi.label(safe&gate_crop)
    gate_anchor_labels=[]
    for gx in [gate['x_m']-.5,gate['x_m']+.5]:
        iy=int(np.argmin(abs(yy-gate['y_m'])));ix=int(np.argmin(abs(xx-gx)))
        gate_anchor_labels.append(int(local_labels[iy,ix]))
    gate_passage=gate_anchor_labels[0]>0 and gate_anchor_labels[0]==gate_anchor_labels[1]
    free_labels,n_free=ndi.label(free);sizes=np.bincount(free_labels.ravel());sizes[0]=0
    largest=int(sizes.argmax());main_free=free_labels==largest
    edge=np.zeros(a.shape,dtype=bool);edge[[0,-1],:]=True;edge[:,[0,-1]]=True
    occupied_labels,n_occ=ndi.label(occupied);occ_sizes=np.bincount(occupied_labels.ravel())
    small_ids=np.flatnonzero((occ_sizes>0)&(occ_sizes<=5));small_ids=small_ids[small_ids>0]
    # Report only small occupied islands fully surrounded by the main free component.
    islands=[]
    for label in small_ids:
        component=occupied_labels==label
        border=ndi.binary_dilation(component)&~component
        if np.all(main_free[border]):islands.append(int(label))
    isolated_cells=int(sum(occ_sizes[label] for label in islands))
    usability=dict(robot_radius_m=args.robot_radius,diameter_m=2*args.robot_radius,
                   clearance_correction_m=r/math.sqrt(2),anchors=anchors,
                   anchors_connected=bool(all_connected),local_gate_passable=bool(gate_passage),
                   gate_width_ge_3_cells=bool(gate['min_cells']>=3),
                   gate_nominal_width_m=1.0,gate_width_minus_nominal_m=gate['width_min_m']-1.,
                   gate_margin_after_diameter_m=gate['width_min_m']-2*args.robot_radius,
                   main_free_cells=int(main_free.sum()),main_free_area_m2=float(main_free.sum()*r*r),
                   other_free_cells=int(free.sum()-main_free.sum()),
                   main_free_cells_at_image_edge=int((main_free&edge).sum()),
                   small_enclosed_occupied_components=len(islands),small_enclosed_occupied_cells=isolated_cells,
                   small_island_max_cells=5,
                   limitation='Connectivity under an assumed circular radius, treating unknown as blocked. Not actual Nav2 execution, not complete false-obstacle detection.')
    route=None;route_xy=None
    if args.route:route,route_xy=route_metrics(args.route)
    graph=None
    if args.graph:
        from eval_pose_graph import evaluate_graph
        graph=evaluate_graph(args.graph,out)
    metrics=dict(map_file=str(args.map),route_file=str(args.route) if args.route else None,
                 map=dict(width_cells=a.shape[1],height_cells=a.shape[0],resolution_m=r,
                          origin=meta['origin'],occupied_thresh=meta['occupied_thresh'],free_thresh=meta['free_thresh']),
                 walls={name:w['metrics'] for name,w in walls.items()},
                 final_wall_bands=bands,gate=gate,usability=usability,route=route,
                 graph=graph,
                 revisit_constraints_verified=bool(graph and graph['revisit_constraints_verified']),
                 loop_closure_activation_verified=False,
                 loop_closure_note='See graph evidence for cross-temporal revisit constraints. MarkerArray does not identify the internal detector callback. Driving a closed route and a single wall band alone are insufficient.')
    (out/'quality_metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
    (out/'regions_used.json').write_text(json.dumps(regions,ensure_ascii=False,indent=2)+'\n')
    with (out/'quality_metrics.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['metric','value','unit'])
        for name,w in walls.items():
            for key in ['rms_m','max_abs_m','band90_m']:writer.writerow([name+'.'+key,w['metrics'][key],'m'])
        for name,key,unit in [('gate','width_min_m','m'),('gate','min_cells','cells')]:writer.writerow([name+'.'+key,gate[key],unit])
        writer.writerow(['final_wall_bands.multiple_band_slices',bands['multiple_band_slices'],'slices'])
        writer.writerow(['local_gate_passable',gate_passage,'boolean'])
        if route:
            for key in ['duration_s','max_tracking_error_m','last_logged_distance_to_start_m','min_front_range_m']:writer.writerow(['route.'+key,route[key],'s' if key.endswith('_s') else 'm'])
    extent=[xx[0]-r/2,xx[-1]+r/2,yy[0]-r/2,yy[-1]+r/2]
    def draw_map(ax):
        ax.imshow(a,cmap='gray',vmin=0,vmax=255,origin='lower',extent=extent,interpolation='nearest')
        ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
    fig,axs=plt.subplots(1,2,figsize=(13,6),constrained_layout=True)
    draw_map(axs[0]);axs[0].set_title('Saved map + measurement regions (pixels unchanged)')
    for name,color in [('south_outer_wall','tab:blue'),('south_inner_wall','tab:orange'),('gate','tab:purple')]:
        x0,x1,y0,y1=regions[name];axs[0].add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,edgecolor=color,lw=1.5,label=name))
    axs[0].legend(fontsize=8,loc='center')
    draw_map(axs[1]);axs[1].contour(xx,yy,safe.astype(float),levels=[.5],colors=['tab:green'],linewidths=.6)
    for name,point in anchors.items():
        axs[1].plot(point['x_m'],point['y_m'],'ro',ms=3);axs[1].annotate(name,(point['x_m'],point['y_m']),fontsize=8)
    axs[1].set_title(f'Circular-radius check: {args.robot_radius:.2f} m; unknown blocked')
    fig.savefig(out/'map_quality_overview.png',dpi=180);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(13,7),constrained_layout=True)
    for col,(name,wall) in enumerate(walls.items()):
        pts=wall['points'];axs[0,col].scatter(pts[:,0],pts[:,1],s=5,label='All occupied pixel centers in ROI')
        along=np.array([wall['along'].min(),wall['along'].max()]);line=wall['center']+along[:,None]*wall['tangent']
        axs[0,col].plot(line[:,0],line[:,1],'r-',label='Orthogonal least-squares line')
        axs[0,col].set_title(name);axs[0,col].set_xlabel('map x (m)');axs[0,col].set_ylabel('map y (m)');axs[0,col].legend(fontsize=8)
        axs[1,col].scatter(wall['along'],wall['residual']*100,s=5)
        axs[1,col].axhline(0,color='r');axs[1,col].set_xlabel('Along fitted wall (m)');axs[1,col].set_ylabel('Normal residual (cm)')
        m=wall['metrics'];axs[1,col].set_title(f"RMS={m['rms_m']*100:.2f}cm; max={m['max_abs_m']*100:.2f}cm; no outlier removal")
    fig.savefig(out/'wall_straightness.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,6),constrained_layout=True);draw_map(ax)
    ax.set_xlim(gate['x_m']-1.1,gate['x_m']+1.1);ax.set_ylim(gate['y_m']-1.1,gate['y_m']+1.1)
    for section in gate['sections']:ax.plot([section['x_m']]*2,[section['y_low_m'],section['y_high_m']],color='tab:blue',lw=2)
    ax.set_title(f"Gate free opening: min {gate['width_min_m']:.2f}m ({gate['min_cells']} cells)\nConservative local passage: {gate_passage}")
    fig.savefig(out/'gate_width.png',dpi=180);plt.close(fig)
    if route_xy is not None:
        fig,ax=plt.subplots(figsize=(8,6),constrained_layout=True)
        ax.plot(route_xy[:,0],route_xy[:,1],lw=1,label='Logged odometry, initial-pose frame')
        ax.plot(*route_xy[0],'go',label='First logged point');ax.plot(*route_xy[-1],'rx',label='Last logged point before stopping')
        ax.set_aspect('equal');ax.grid(alpha=.3);ax.legend(fontsize=8);ax.set_xlabel('initial odom x (m)');ax.set_ylabel('initial odom y (m)')
        ax.set_title('Route coverage; not overlaid onto the different map frame')
        fig.savefig(out/'route_coverage.png',dpi=180);plt.close(fig)
    outer=walls['south_outer_wall']['metrics'];inner=walls['south_inner_wall']['metrics']
    lines=['# M3-2 地图质量评估（本机可复现）','',
           '## 输入与方法','',
           f'- 地图：`{args.map}`；尺寸 {a.shape[1]} × {a.shape[0]}，分辨率 {r:.3f} m/格。',
           f"- 依据 map.yaml 分类：free < {meta['free_thresh']}，occupied > {meta['occupied_thresh']}；其余为未知。",
           '- 使用像素中心坐标。测量区域见 `regions_used.json` 和总览图；默认区域仅适用于本次环形场景及初始坐标约定。',
           '- 墙线采用全部占据像素的正交最小二乘拟合；不删除离群点、不修图。窄门位置从区域内门柱占据像素自动定位。',
           '- map 与 odom 是不同坐标系；未把 odom 轨迹直接叠加到地图上，也未用 Gazebo 真值修正地图。','',
           '## 1. 墙直线度','',
           '| 测量区域 | 占据像素数 | RMS 法向残差 | 最大绝对残差 | 中央 90% 法向带宽 |',
           '| --- | ---: | ---: | ---: | ---: |']
    for name,w in walls.items():
        m=w['metrics'];lines.append(f"| {name} | {m['points']} | {m['rms_m']:.4f} m | {m['max_abs_m']:.4f} m | {m['band90_m']:.4f} m |")
    lines += ['', '选择南侧内外墙中央直线段，测量框避开实际地图中的转角。残差同时包含墙面栅格厚度、量化及建图不一致，不是纯定位误差。',
              '初始候选测量框曾包含内墙左转角，产生约 0.37 m 残差；经原图核对后，将测量框收窄至中央直线段。该 0.37 m 不能当作建图误差。未对当前框内像素作离群点删除。图：`wall_straightness.png`。','',
              '## 2. 最终地图的回环区域重影筛查','',
              f"- 对南侧外墙按 0.25 m 分段，共检查 {bands['test_slices']} 段；满足双占据带判据的有 {bands['multiple_band_slices']} 段。",
              f"- 该墙中央 90% 法向带宽为 {outer['band90_m']:.4f} m；这不是两次扫描的墙间距。",
              '- 判据：每条带至少 2 个占据像素，带间至少一个空法向栅格，带中心距离至少 2 格（0.10 m）。',
              '- 若未检出第二条带，只能说按此判据未分辨出双带；不能填“两次墙间距=0”，也不能证明不存在亚栅格漂移。',
              (f"- **位姿图补充证据**：{graph['nodes']} 个节点、{graph['unique_edges']} 条边，其中 {graph['early_late_edges']} 条连接早期与末期观测；重访区域跨时序约束验证：{graph['revisit_constraints_verified']}。详见 `pose_graph_report.md`、`pose_graph_evidence.png`。MarkerArray 不区分内部回环检测函数。" if graph else '- **SLAM 插入回环约束尚未由本程序验证**：路线绕回、最终单墙带是质量证据，实际触发仍需 SLAM 日志或位姿图证据。'),'',
              '## 3. 地图可用性','',
              f"- 窄门最小连续已知自由宽度：**{gate['width_min_m']:.2f} m / {gate['min_cells']} 格**；名义设计宽度 1.00 m，差 {gate['width_min_m']-1:+.2f} m。该差值包含栅格量化。",
              f"- 评估用圆形半径 {args.robot_radius:.2f} m（直径 {2*args.robot_radius:.2f} m），门宽减直径余量 {gate['width_min_m']-2*args.robot_radius:.2f} m。",
              f'- 所有走廊代表点在膨胀后自由空间中连通：**{all_connected}**。',
              f'- 限定门口局部区域、排除绕环另一侧的替代路径后，门两侧连通：**{gate_passage}**。',
              f'- 距离场额外扣除半个栅格对角线 {r/math.sqrt(2):.4f} m；未知区域按障碍处理。',
              f"- 主自由空间面积 {usability['main_free_area_m2']:.3f} m²；与图像边界接触自由格 {usability['main_free_cells_at_image_edge']}。这只检查已知自由空间有无通向图像边缘的通道，不代表现实边界完全无缺口。",
              f"- 未连接主走廊的自由格 {usability['other_free_cells']}；总览图保留外墙附近条纹。它们不是自动认定的伪障碍。",
              f'- 完全被主自由空间包围、面积不超过 5 格的小占据岛：{len(islands)} 个 / {isolated_cells} 格。只筛查这类伪障碍候选，不能据此认定所有障碍真假。',
              '- 图：`map_quality_overview.png`、`gate_width.png`。这项是静态保守通行性检查，**不是已经运行 Nav2 导航**。','']
    if route:
        lines += ['## 4. 巡场记录','',
                  f"- CSV {route['rows']} 行，记录时长 {route['duration_s']:.2f} s。",
                  f"- 最大参考路线跟踪误差 {route['max_tracking_error_m']:.4f} m；前方最近记录距离 {route['min_front_range_m']:.4f} m。",
                  f"- 最后一条记录距第一条 {route['last_logged_distance_to_start_m']:.4f} m。脚本约在 0.15 m 起点容差内停止，且 CSV 未写入最终停车样本，因此此值不能当作最终闭环漂移。",
                  '- `Loop complete` 由运行终端确认；本程序仅校验 CSV，不能从缺少结束事件的 CSV 独立认证正常退出。',
                  '- 图：`route_coverage.png`。','']
    lines += ['## 仍需补齐的验证','',
              ('- 位姿图已检查：存在跨早期与末期的重访约束；不声称已识别内部检测函数。' if graph and graph['revisit_constraints_verified'] else '- SLAM 实际回环触发的日志或位姿图证据。'),
              '- 用 map_server 加载保存地图的验证。',
              '- rosbag 完整性检查；本程序没有读取或验证 bag。',
              '- 最终 README、依赖与复现步骤；可选的单参数对比实验尚未开展。','']
    (out/'quality_report.md').write_text('\n'.join(lines),encoding='utf-8')
    print(f"Outer wall: RMS={outer['rms_m']:.6f}m; max={outer['max_abs_m']:.6f}m")
    print(f"Inner wall: RMS={inner['rms_m']:.6f}m; max={inner['max_abs_m']:.6f}m")
    print(f"Gate: min={gate['width_min_m']:.3f}m / {gate['min_cells']} cells; local passage={gate_passage}")
    print(f"Wall multiple-band slices: {bands['multiple_band_slices']}/{bands['test_slices']}")
    print(f"Inflated corridor anchors connected: {all_connected}")
    print(f"Graph revisit constraints verified: {graph['revisit_constraints_verified']} ({graph['early_late_edges']} early/late edges)" if graph else 'Loop-constraint insertion is NOT verified by this static-map evaluation.')
    print(f'Outputs: {out.resolve()}')

if __name__=='__main__':main()
