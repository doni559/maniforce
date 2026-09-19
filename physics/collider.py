from pygame import draw

from typing import Tuple, List

from .utils import Vector
from settings import HEIGHT, EPS

class Collider():
    def __init__(self, collider_type : str , center : Vector,**kwargs):
        possible_types = [
            "Box",
            "Circle",
            "Polygon"
        ]

        if not collider_type.capitalize() in possible_types:
            raise ValueError("Collider type is unknown") 
        self.type = collider_type.capitalize()
        self.center = center
        if self.type == "Circle":
            self.radius = kwargs["radius"]
        if self.type in ["Box", "Polygon"]:
            self.relative_corners = kwargs["corners"]
            world_corners = [Vector(center.x+point.x, center.y+point.y) for point in self.relative_corners]
            self.corners = world_corners
        
    def draw(self, screen, color, center : Tuple[int] | None = None):
        if (self.type == "Circle"):
            draw.circle(screen, color, center, radius=self.radius)
        if (self.type == "Box" or self.type == "Polygon"):
            draw.polygon(screen,color, [point.convert_to_screen_cords() for point in self.corners])

    def calculate_deformation(self, another_collider, object_pos , another_object_pos) -> Tuple[Vector, List[dict]]:
        another_collider : Collider = another_collider
        if (self.type == "Circle" and another_collider.type == "Circle"):
            collision_distance = sum([self.radius, another_collider.radius])
            distance : Vector= (another_object_pos - object_pos)
            deformation= collision_distance-distance.get_length()
            normal = distance.normalise()
            if distance.scalar_multiply(normal) >0:
                normal*= -1
            contact_points=[]
            if deformation > 0:
                contact_points.append({
                    "pos":object_pos-normal*self.radius,
                    "deformation":deformation
                })
            return normal, contact_points
        if (self.type == "Circle" and another_collider.type in ["Polygon", "Box"]):
            normals : List[Vector]=[]
            penetrations : List[float] = []
            for i in range(0, len(another_collider.corners)):
                point = another_collider.corners[i]
                if i == (len(another_collider.corners)-1):
                    next_point= another_collider.corners[0]
                else:
                    next_point= another_collider.corners[i+1]
                side= next_point-point
            
                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)
            for i in range(0, len(another_collider.corners)):
                normal= (another_collider.corners[i]-object_pos).normalise()
                normals.append(normal)
            
            #projecting objects on normals and writing intervals
            for normal in normals:
                projections : List[Vector]=[]
                center= self.center.scalar_multiply(normal)
                t_interval=[
                    center-self.radius,
                    center+self.radius
                ]
                projections.clear()
                for point in another_collider.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                a_interval=[
                    min(projections),
                    max(projections)
                ]
                penetration=min(a_interval[1]-t_interval[0], t_interval[1]-a_interval[0])
                #check if projections are overlapping
                if penetration < 0:
                    #then no collision, return normal=0 (no need), deformation=0
                    return Vector(0,0), [] 
                penetrations.append(penetration)  
            deformation=min(penetrations)
            index=penetrations.index(deformation)
            normal = Vector(normals[index].x, normals[index].y)
            contact_points=[]
            distance : Vector = another_object_pos - object_pos
            if distance.scalar_multiply(normal) > 0:
                normal*=-1
            if deformation > 0:
                contact_points.append(
                    {
                        "pos":object_pos-normal*self.radius,
                        "deformation":deformation
                    }
                )
            return normal, contact_points
        if (self.type in ["Box", "Polygon"] and another_collider.type == "Circle"):
            normals : List[Vector]=[]
            penetrations : List[float] = []
            for i in range(0, len(self.corners)):
                point = self.corners[i]
                if i == (len(self.corners)-1):
                    next_point= self.corners[0]
                else:
                    next_point= self.corners[i+1]
                side= next_point-point
        
                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)
            for i in range(0, len(self.corners)):
                normal= (self.corners[i]-another_object_pos).normalise()
                normals.append(normal)
            #projecting objects on normals and writing intervals
            for normal in normals:
                projections : List[Vector]=[]

                center= another_collider.center.scalar_multiply(normal)
                t_interval=[
                    center-another_collider.radius,
                    center+another_collider.radius
                ]
                projections.clear()
                for point in self.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                a_interval=[
                    min(projections),
                    max(projections)
                ]
                penetration=min(a_interval[1]-t_interval[0], t_interval[1]-a_interval[0])
                #check if projections are overlapping
                if penetration < 0:
                    #then no collision, return normal=0 (no need), deformation=0
                    return Vector(0,0), [] 
                penetrations.append(penetration)  
            deformation=min(penetrations)
            index=penetrations.index(deformation)

            normal = Vector(normals[index].x, normals[index].y)
            contact_points=[]
            distance : Vector = another_object_pos- object_pos
            if distance.scalar_multiply(normal) > 0:
                normal*=-1
            if deformation > 0:
                contact_points.append(
                    {
                        "pos":another_object_pos+normal*another_collider.radius,
                        "deformation":deformation
                    }
                )       
            return normal, contact_points

        if (self.type in ["Box", "Polygon"] and another_collider.type in ["Box", "Polygon"]):
            #SAT
            #finding normals
            normals : List[Vector]=[]
            penetrations : List[float] = []
            for i in range(0, len(another_collider.corners)):
                point = another_collider.corners[i]
                if i == (len(another_collider.corners)-1):
                    next_point= another_collider.corners[0]
                else:
                    next_point= another_collider.corners[i+1]
                side= next_point-point

                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)

            for i in range(0, len(self.corners)):
                point = self.corners[i]
                if i == (len(self.corners)-1):
                    next_point= self.corners[0]
                else:
                    next_point= self.corners[i+1]

                side=  next_point-point
                normal = Vector(side.y, -side.x).normalise()
                normals.append(normal)
            
            #projecting objects on normals and writing intervals
            for normal in normals:
                projections : List[Vector]=[]
                for point in self.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                t_interval=[
                    min(projections),
                    max(projections)
                ]
                projections.clear()
                for point in another_collider.corners:
                    projected_point=point.scalar_multiply(normal)
                    projections.append(projected_point)
                a_interval=[
                    min(projections),
                    max(projections)
                ]
                penetration=min(a_interval[1]-t_interval[0], t_interval[1]-a_interval[0])
                #check if projections are overlapping
                if penetration < 0:
                    #then no collision, return normal=0 (no need), deformation=0
                    return Vector(0,0), [] 
                penetrations.append(penetration)  
            deformation=min(penetrations)
            
            index=penetrations.index(deformation)
            normal = Vector(normals[index].x, normals[index].y)
            
            distance : Vector = another_object_pos - object_pos
            if index in range(0 , len(another_collider.corners)):
                reference_index = index
                reference = another_collider
                incident = self
                
                max_scalar_normal=float("-inf")
                if distance.scalar_multiply(normal) > 0:
                    normal *= -1

                for i in range(0, len(reference.corners)):
                    
                    if normals[i].scalar_multiply(normal) > max_scalar_normal:
                        max_scalar_normal = normals[i].scalar_multiply(normal)
                        reference_index=i
                
            else:
                reference_index=index-len(another_collider.corners)
                reference = self
                incident = another_collider
                
                max_scalar_normal=float("-inf")
                if distance.scalar_multiply(normal) < 0:
                    normal *= -1
                
                for i in range(len(another_collider.corners), len(another_collider.corners)+len(self.corners)):
                    if normals[i].scalar_multiply(normal) > max_scalar_normal:
                        max_scalar_normal = normals[i].scalar_multiply(normal)
                        reference_index=i-len(another_collider.corners)
    
            reference_p0=reference.corners[reference_index]
            reference_p1=reference.corners[(reference_index + 1) % len(reference.corners)]

            min_scalar_normal = float("inf")
            incident_index = 0
            if reference is self:
                for k in range(0, len(another_collider.corners)):
                    n1=normals[k]
                    scalar=n1.scalar_multiply(normal)
                    if scalar < min_scalar_normal:
                        min_scalar_normal = scalar
                        incident_index= k
            else:
                for k in range(0, len(self.corners)):
                    n1=normals[len(another_collider.corners)+k]
                    scalar=n1.scalar_multiply(normal)
                    if scalar < min_scalar_normal:
                        min_scalar_normal = scalar
                        incident_index= k
            incident_p0=incident.corners[incident_index]
            incident_p1=incident.corners[(incident_index + 1) % len(incident.corners)]

            tangent_L=(reference_p1-reference_p0).get_length()
            tangent : Vector = (reference_p1-reference_p0).normalise()

            s0=(incident_p0-reference_p0).scalar_multiply(tangent)
            s1=(incident_p1-reference_p0).scalar_multiply(tangent)

            if abs(s1 - s0) < EPS:
                clipped_p0, clipped_p1 = incident_p0, incident_p1
            else:
                left_border = -s0/(s1-s0)
                right_border = (tangent_L-s0)/(s1-s0)
                if (s0 < 0 and s1 < 0) or (s0 > tangent_L and s1 > tangent_L):
                    return Vector(0,0), []
                if s0 < 0:
                    clipped_p0= incident_p0+(incident_p1-incident_p0)*left_border
                elif s0 > tangent_L:
                    clipped_p0= incident_p0+(incident_p1-incident_p0)*right_border
                else:
                    clipped_p0=incident_p0
                if s1 <0:
                    clipped_p1 = incident_p0 +(incident_p1-incident_p0) *  left_border
                elif s1 > tangent_L:
                    clipped_p1 = incident_p0 +(incident_p1-incident_p0) *  right_border

                else:
                    clipped_p1 = incident_p1

            deformation_p0 = -(clipped_p0-reference_p0).scalar_multiply(normal)
            deformation_p1 = -(clipped_p1-reference_p0).scalar_multiply(normal)
            contact_points =[]
            if deformation_p0 >= 0:
                contact_points.append({
                    "pos": clipped_p0,
                    "deformation":deformation_p0
                })
            if deformation_p1 >= 0:
                
                contact_points.append({
                    "pos": clipped_p1,
                    "deformation":deformation_p1
                })
            if distance.scalar_multiply(normal) > 0:
                normal *= -1
            return normal, contact_points
