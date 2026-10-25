#!/usr/bin/env python3
"""Export Robot.STEP as visual-only meshes fitted to the unchanged simulation.
Requires cadquery-ocp==8.0.1.1.0 and numpy. Run with Robot.STEP as argv[1].
All distances below are millimetres until the final metre conversion.
"""
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TopLoc import TopLoc_Location
import json,sys

doc=TDocStd_Document(TCollection_ExtendedString('cad'))
r=STEPCAFControl_Reader(); r.SetNameMode(True); r.SetColorMode(True)
assert r.ReadFile(sys.argv[1]).name=='IFSelect_RetDone'
assert r.Transfer(doc)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots=TDF_LabelSequence(); st.GetFreeShapes(roots)
def name(label):
    n=TDataStd_Name()
    return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),n) else '?'
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
import numpy as np
parts=[]
def extract(label,loc=TopLoc_Location(),path=[],names=[]):
    ref=TDF_Label()
    if st.IsReference_s(label):
        loc=loc.Multiplied(st.GetLocation_s(label)); st.GetReferredShape_s(label,ref); label=ref
    names=names+[name(label)]
    children=TDF_LabelSequence()
    if st.GetComponents_s(label,children):
        for i in range(1,children.Length()+1):extract(children.Value(i),loc,path+[i],names)
        return
    shape=st.GetShape_s(label).Moved(loc)
    BRepMesh_IncrementalMesh(shape,0.08,False,0.25,True)
    exp=TopExp_Explorer(shape,TopAbs_FACE); triangles=[]
    while exp.More():
        face=TopoDS.Face(exp.Current()); location=TopLoc_Location()
        mesh=BRep_Tool.Triangulation_s(face,location)
        if mesh is not None:
            tr=location.Transformation()
            nodes=np.array([[p.X(),p.Y(),p.Z()] for p in [mesh.Node(i).Transformed(tr) for i in range(1,mesh.NbNodes()+1)]])
            ids=np.array([mesh.Triangle(i).Get() for i in range(1,mesh.NbTriangles()+1)])-1
            if face.Orientation()==TopAbs_REVERSED: ids=ids[:,[0,2,1]]
            triangles.append(nodes[ids])
        exp.Next()
    arr=np.concatenate(triangles)
    parts.append(dict(path=path,names=names,triangles=arr))
for i in range(1,roots.Length()+1):extract(roots.Value(i))

import pathlib,struct,hashlib,os
out=pathlib.Path(__file__).resolve().parents[1]/'src/biped_description/meshes/cad'
out.mkdir(parents=True,exist_ok=True)
# Original CAD joints: sole 0; shin/ankle mounting plane 67.189;
# knee shaft 127.189; hip pitch shaft 187.189. The sim has co-located
# ankle pitch/roll axes, so both CAD ankle subassemblies follow foot.
CAD_Z=np.array([0.,67.189,127.189,187.189,300.])
SIM_Z=np.array([0.,150.,350.,550.,775.622])
# Keep CAD left/right spacing, set its 99.029 mm leg centres to 200 mm.
def fit(v):
    q=v.copy()
    q[...,0]=(v[...,0]-.66)*2.
    q[...,1]=(v[...,1]+49.5035)*(200./99.029)
    # Preserve the original imported assembly orientations.
    z=v[...,2]
    q[...,2]=np.interp(z,CAD_Z,SIM_Z)-600.
    q[...,2]=np.where(z<0,z*(150./67.189)-600.,q[...,2])
    return q/1000.

GROUPS={
 'base_link':([8,11],[0,0,0]),
 'platform_link':([9],[0,0,.065]),
 'left_hip_pitch_link':([6,7],[0,.1,-.05]),
 'right_hip_pitch_link':([10,13],[0,-.1,-.05]),
 'left_thigh':([4,5],[0,.1,-.05]),
 'right_thigh':([12,14],[0,-.1,-.05]),
 'left_shin':([3],[0,.1,-.25]),
 'right_shin':([15],[0,-.1,-.25]),
 'left_foot':([1,2],[0,.1,-.45]),
 'right_foot':([16,17],[0,-.1,-.45]),
}

def stl(path,triangles):
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths=np.linalg.norm(normals,axis=1)
    keep=lengths>1e-14; triangles=triangles[keep]; normals=normals[keep]/lengths[keep,None]
    data=np.zeros(len(triangles),dtype=[('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
    data['normal']=normals; data['vertices']=triangles
    with path.open('wb') as f:
        f.write(b'Robot.STEP visual geometry; millimetres converted to metres'.ljust(80,b' '))
        f.write(struct.pack('<I',len(data))); f.write(data.tobytes())
    return len(data)

manifest={'source':os.path.relpath(pathlib.Path(sys.argv[1]).resolve(), out),'source_sha256':hashlib.sha256(pathlib.Path(sys.argv[1]).read_bytes()).hexdigest(),'cad_colour_srgb':[.792156862745098,.819607843137255,.9333333333333333,1.], 'units':'metres', 'tessellation_deflection_mm':.08, 'tessellation_angle_radians':.25,'cad_z_landmarks_mm':CAD_Z.tolist(),'sim_z_landmarks_mm':SIM_Z.tolist(),'physics_baseline':'8d6433f','note':'Visuals fitted to unchanged sim joints; not a dimensionally identical CAD model. CAD ankle yoke and servo follow the foot together because sim ankle axes are co-located.','meshes':{}}
assigned=[]
for link,(indices,origin) in GROUPS.items():
    selected=[p for p in parts if p['path'][0] in indices]
    assigned += [tuple(p['path']) for p in selected]
    triangles=fit(np.concatenate([p['triangles'] for p in selected]))-np.array(origin)
    count=stl(out/(link+'.stl'),triangles)
    manifest['meshes'][link]={'triangles':count,'cad_top_level_components':indices,'link_origin_m':origin,'bounds_m':[triangles.reshape(-1,3).min(axis=0).tolist(),triangles.reshape(-1,3).max(axis=0).tolist()], 'parts':[{'path':p['path'],'name':p['names'][-1]} for p in selected]}
    print(link,len(selected),'parts',count,'triangles')
assert len(assigned)==len(parts)==len(set(assigned)), 'Every CAD instance must be exported exactly once'
(out/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
print('Exported',len(parts),'CAD instances into',len(GROUPS),'visual meshes')
