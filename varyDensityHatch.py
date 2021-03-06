import rhinoscriptsyntax as rs
import math as m

def testIntersection(crv01,crv02):
    try:
        rs.CurveCurveIntersection(crv01,crv02)
    except None:
        return False
    return True
    
##################################
#GENHATCHX takes the bounding box as a starting point. 
#It defines a reference line on one axis of the
#bounding box, then moves the reference point's end point 
#until it hits the desired hatch angle. Then it calculates
#the necessary number of hatches to cover the profile, arrays
#them, splits the arrayed hatches with the profile
#and deletes all the hatches whose midpoints lie outside
#of the profile
##################################


def genHatchX(profile,attPt,strength,ang,gap,min):
    crvs = []
    hatch = []
    sections = []
    box = rs.BoundingBox(profile)
    start = rs.CurveStartPoint(profile)
    #creates ref VECX as a starting axis and direction of copying
    vecX = rs.VectorUnitize(rs.VectorCreate(box[1],box[0]))
    #NUMX is the req. distance to move the end point of the line to form
    #the correct angle of the hatch
    numX = rs.Distance(box[0],box[3])*m.tan(ang*m.pi/180)
    axisX = rs.PointAdd(box[3],-vecX*numX)
    #X is the first hatch line that will be copied to cover profile
    x = rs.AddCurve([box[0],axisX],1)
    #CORRECT lifts the hatch line to the height of the profile
    correct = [0,0,start[2]-box[0][2]]
    x = rs.MoveObject(x,correct)
    #MAX is the dimension that the hatch will have to move over to cover
    #entire profile
    limit = numX+rs.Distance(box[0],box[1])
    #the for loop below copies each curve down to profile
    crv = x
    copy = 0
    while copy<limit:
        param = rs.CurveClosestPoint(crv,attPt)
        close = rs.EvaluateCurve(crv,param)
        val = rs.Distance(attPt,close)/strength
        if val>1:
            val = 1
        if val<min:
            val = min
        copy = copy+val*gap
        crv = rs.CopyObject(x,copy*vecX)
        crvs.append(crv)
    for i in range(len(crvs)):
        sections.append(splitCrv(crvs[i],[profile]))
    #The splitCrv creates lists of curves that are then checked to 
    #see if they lie within the profile (by looking at their midpoint)
    for i in range(len(sections)):
        for j in range(len(sections[i])):
            if rs.PointInPlanarClosedCurve(rs.CurveMidPoint(sections[i][j]),profile):
                section = sections[i][j]
            else:
                rs.DeleteObject(sections[i][j])
            hatch.append(sections[i][j])
    rs.DeleteObjects(crvs)
    rs.DeleteObject(x)
    return crvs

##################################
# GENHATCHY function works the same way as the genHatchX
# it just uses the y axis of the bounding box as a reference
##################################

def genHatchY(profile,attPt,strength,ang,gap,min):
    crvs = []
    hatch = []
    sections = []
    box = rs.BoundingBox(profile)
    start = rs.CurveStartPoint(profile)
    vecY = rs.VectorUnitize(rs.VectorCreate(box[3],box[0]))
    numY = rs.Distance(box[0],box[1])*m.tan(ang*m.pi/180)
    axisY = rs.PointAdd(box[1],-vecY*numY)
    y = rs.AddCurve([box[0],axisY],1)
    correct = [0,0,start[2]-box[0][2]]
    y = rs.MoveObject(y,correct)
    limit=numY+rs.Distance(box[0],box[3])
    crv = y
    copy = 0
    while copy<limit:
        param = rs.CurveClosestPoint(crv,attPt)
        close = rs.EvaluateCurve(crv,param)
        val = rs.Distance(attPt,close)/strength
        if val>1:
            val = 1
        if val<min:
            val = min
        copy = copy+val*gap
        crv = rs.CopyObject(y,copy*vecY)
        crvs.append(crv)
    for i in range(len(crvs)):
        sections.append(splitCrv(crvs[i],[profile]))
    for i in range(len(sections)):
        for j in range(len(sections[i])):
            if rs.PointInPlanarClosedCurve(rs.CurveMidPoint(sections[i][j]),profile):
                section = sections[i][j]
            else:
                rs.DeleteObject(sections[i][j])
            hatch.append(sections[i][j])
    rs.DeleteObjects(crvs)
    rs.DeleteObject(y)
    return crvs

def splitCrv(profile,splitters):
    sects = []
    #loops through each cutting curve
    for i in range(len(splitters)):
        #checks to see if curves intersect
        if rs.CurveCurveIntersection(splitters[i],profile)!=None:
            params = []
            intersect = rs.CurveCurveIntersection(profile,splitters[i])
            #loops through the result, taking parameter on profile line 
            for j in range(len(intersect)):
                params.append(intersect[j][5])
            #splits the profile curve at those parameters
            sects = rs.SplitCurve(profile,params)
    return sects

def Main():
    profile = rs.GetObject("select profile",rs.filter.curve)
    attPt = rs.GetObject("select point",rs.filter.point)
    angX = rs.GetReal("enter angle of hatch in first direction (degrees)", 30)
    angY = rs.GetReal("enter angle of hatch in second direction (degrees)", 40)
    spacingX = rs.GetReal("enter desired spacing of hatch in first direction",3)
    spacingY = rs.GetReal("enter desired spacing of hatch in second direction",3)
    strength = rs.GetReal("enter desired range for attractor",30)
    minX = rs.GetReal("enter minimum spacing in first direction",.1*spacingX)
    minY = rs.GetReal("enter minimum spacing in second direction",.1*spacingY)
    genHatchX(profile,attPt,strength,angX,spacingX,minX)
    genHatchY(profile,attPt,strength,angY,spacingY,minY)

Main()