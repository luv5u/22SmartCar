#!/usr/bin/env python3
"""Read-only evaluation of slam_toolbox graph_visualization MarkerArray YAML.
Checks graph structure, not the internal detector's call path or residuals.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def evaluate_graph(path,out):
    documents=[d for d in yaml.safe_load_all(path.read_text()) if d]
    if len(documents)!=1:raise ValueError('Expected exactly one nonempty graph snapshot')
    markers=documents[0].get('markers',[])
    if not markers or any(not isinstance(m,dict) for m in markers):
        raise ValueError('Truncated/invalid markers. Export with ros2 topic echo --once --full-length')
    nodes={}
    for m in markers:
        if m.get('ns')=='slam_toolbox' and m.get('action')==0:
            if m.get('header',{}).get('frame_id')!='map':raise ValueError('Graph is not in map frame')
            p=m['pose']['position'];key=int(m['id'])
            if key in nodes:raise ValueError('Duplicate node ID')
            nodes[key]=[float(p['x']),float(p['y'])]
    if len(nodes)<3:raise ValueError('Not enough node markers; interactive mode may hide nodes')
    ids=np.array(sorted(nodes));xy=np.array([nodes[i] for i in ids])
    if not np.isfinite(xy).all():raise ValueError('Nonfinite graph coordinates')
    if not np.all(np.diff(ids)==1):raise ValueError('Noncontiguous node IDs; this one-sensor temporal-order check requires contiguous IDs')
    tree=cKDTree(xy);pairs=set();max_error=0.;raw_edges=0
    for m in markers:
        if m.get('ns')!='slam_toolbox_edges' or m.get('type')!=5 or m.get('action')!=0:continue
        points=m.get('points',[])
        if any(not isinstance(p,dict) for p in points) or len(points)%2:
            raise ValueError('Truncated/invalid edge endpoint array')
        for p,q in zip(points[::2],points[1::2]):
            target=np.array([[p['x'],p['y']],[q['x'],q['y']]],float)
            if not np.isfinite(target).all():raise ValueError('Nonfinite edge coordinates')
            distance,index=tree.query(target,k=2)
            if np.max(distance[:,0])>1e-6:raise ValueError('An edge endpoint does not match a node within 1 micrometer')
            if np.any(distance[:,1]<1e-6):raise ValueError('Ambiguous coincident nodes; endpoint IDs cannot be recovered uniquely')
            max_error=max(max_error,float(np.max(distance[:,0])))
            a,b=sorted([int(ids[index[0,0]]),int(ids[index[1,0]])])
            if a==b:raise ValueError('Self edge found')
            pairs.add((a,b));raw_edges+=1
    if not pairs:raise ValueError('No graph edges found')
    # Connected components without assuming sequential edges exist.
    adjacency={int(i):set() for i in ids}
    for a,b in pairs:adjacency[a].add(b);adjacency[b].add(a)
    remaining=set(adjacency);components=0
    while remaining:
        stack=[remaining.pop()];components+=1
        while stack:
            for neighbor in adjacency[stack.pop()]:
                if neighbor in remaining:remaining.remove(neighbor);stack.append(neighbor)
    count=len(ids);early_limit=ids[0]+.15*(ids[-1]-ids[0]);late_limit=ids[0]+.85*(ids[-1]-ids[0])
    cross=[(a,b) for a,b in sorted(pairs) if a<=early_limit and b>=late_limit and b-a>=.75*(ids[-1]-ids[0])]
    sequential=[(a,b) for a,b in pairs if b-a==1]
    metrics=dict(source=str(path),frame='map',nodes=count,node_id_min=int(ids[0]),node_id_max=int(ids[-1]),
                 unique_edges=len(pairs),raw_edges=raw_edges,sequential_edges=len(sequential),
                 nonsequential_edges=len(pairs)-len(sequential),connected_components=components,
                 cycle_rank=len(pairs)-count+components,early_late_edges=len(cross),early_late_pairs=[list(p) for p in cross],
                 max_endpoint_match_error_m=max_error,
                 first_last_node_distance_m=float(np.linalg.norm(xy[-1]-xy[0])),
                 revisit_constraints_verified=bool(cross and components==1),
                 detector_callback_verified=False,
                 criterion='One sensor with contiguous acquisition-order IDs. Early/late edges connect the first/last 15% of IDs and span >=75% of ID range.',
                 limitation='Marker edges have no constraint type, timestamp provenance, covariance or optimizer residual. Confirms graph connections across the revisit, not which internal loop detector function added them. Node endpoint distance is not the final vehicle closure error.')
    out.mkdir(parents=True,exist_ok=True)
    (out/'pose_graph_metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
    with (out/'pose_graph_edges.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['source_id','target_id','id_gap','early_late_constraint','corrected_endpoint_distance_m'])
        for a,b in sorted(pairs):w.writerow([a,b,b-a,(a,b) in cross,float(np.linalg.norm(np.array(nodes[a])-nodes[b]))])
    fig,axs=plt.subplots(1,2,figsize=(12,6),constrained_layout=True)
    segments=[(nodes[a],nodes[b]) for a,b in sorted(pairs)]
    for ax in axs:
        ax.add_collection(LineCollection(segments,colors='0.75',linewidths=.7))
        plot=ax.scatter(xy[:,0],xy[:,1],c=ids,s=8,cmap='viridis',zorder=3)
        if cross:ax.add_collection(LineCollection([(nodes[a],nodes[b]) for a,b in cross],colors='crimson',linewidths=1.5,zorder=4))
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.autoscale()
    axs[0].set_title(f'{count} nodes, {len(pairs)} edges; red = early/late constraints')
    near_ids=sorted(set([k for pair in cross for k in pair])) if cross else list(ids[:8])
    near=np.array([nodes[k] for k in near_ids]);lo=near.min(0)-.4;hi=near.max(0)+.4
    axs[1].set_xlim(lo[0],hi[0]);axs[1].set_ylim(lo[1],hi[1]);axs[1].set_title('Revisited region: acquisition-order node IDs')
    representative = sorted(set(k for pair in ([cross[0], cross[len(cross)//2], cross[-1]] if cross else []) for k in pair))
    for key in representative:
        axs[1].annotate(str(key),nodes[key],xytext=(3,10 if key<=early_limit else -15),textcoords='offset points',fontsize=9)
    fig.colorbar(plot,ax=axs[0],label='Node ID',shrink=.65)
    fig.savefig(out/'pose_graph_evidence.png',dpi=180);plt.close(fig)
    report=['# 位姿图中的重访约束证据','',
            f"- 输入：`{path}`；完整 MarkerArray，无数组截断。",
            f"- 节点 {count} 个，ID {ids[0]}–{ids[-1]}；唯一边 {len(pairs)} 条。",
            f'- 相邻节点边 {len(sequential)} 条；非相邻节点边 {len(pairs)-len(sequential)} 条。',
            f'- 连通分量 {components}；独立环数量 E−N+C = {metrics["cycle_rank"]}。',
            f'- 跨早期/末期观测的连接 {len(cross)} 条；端点与节点匹配最大误差 {max_error:.8f} m。','',
            '| 早期节点 | 末期节点 | ID 间隔 |','| ---: | ---: | ---: |']
    report += [f'| {a} | {b} | {b-a} |' for a,b in cross]
    report += ['', '**结论：** '+('重访区域已在位姿图中建立跨时序约束，证据强于仅观察到车辆绕回。' if metrics['revisit_constraints_verified'] else '未按当前判据验证早期/末期重访约束。'),'',
               '这些边把顺序链的早期和末期连接成环。在本次无人工改图的运行中，可作为回环区域已加入图优化约束的证据。MarkerArray 没有边的类别或内部函数调用记录，不能进一步断言每一条都来自某个特定回环检测函数，也不能把 ID 间隔当作秒数。',
               f'首末图节点修正后坐标距离 {metrics["first_last_node_distance_m"]:.4f} m，仅描述两个扫描图节点；不等于真实车身闭环误差。',
               '图：`pose_graph_evidence.png`；全部连接：`pose_graph_edges.csv`；统计：`pose_graph_metrics.json`。','']
    (out/'pose_graph_report.md').write_text('\n'.join(report))
    return metrics


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--graph',type=Path,required=True);p.add_argument('--out',type=Path,default=Path('results'));a=p.parse_args()
    m=evaluate_graph(a.graph,a.out)
    print(f"Graph: {m['nodes']} nodes, {m['unique_edges']} edges; early/late constraints={m['early_late_edges']}; verified={m['revisit_constraints_verified']}")

if __name__=='__main__':main()
