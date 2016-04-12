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


def genHatchX(profile,attPts,strength,ang,gap,min):
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
    for i in range(int(limit/gap)):
        crvs.append(rs.CopyObject(x,vecX*i*gap))
    for i in range(len(crvs)):
        relevant = []
        for j in range(len(attPts)):
            param = rs.CurveClosestPoint(crvs[i],attPts[j])
            close = rs.EvaluateCurve(crvs[i],param)
            if rs.Distance(attPts[j],close)<strength*2:
                relevant.append(attPts[j])
        if len(relevant)>0:
            divPts = rs.DivideCurve(crvs[i],20)
            for j in range(len(divPts)):
                sum = 0
                length = 0
                vecSum = [0,0,0]
                for k in range(len(relevant)):
                    attPt = relevant[k]
                    added = rs.Distance(attPt,divPts[j])/strength
                    if added<1:
                        vecSum = rs.PointAdd(vecSum,rs.VectorCreate(attPt,divPts[j]))
                        sum = sum+added
                        length = length+1
                if length!=0:
                    val = 1-sum/length
                    vec = rs.VectorScale(vecSum,val/length)
<<<<<<< HEAD
                    if j!=0 or j!=len(divPts)-1:
=======
                    if j!=0 and j!=len(divPts)-1:
>>>>>>> 59bfeb7955d50ab4cce208edcc2e618bf867223a
                        divPts[j]=rs.PointAdd(divPts[j],vec*(1-val))
            crv = rs.AddCurve(divPts)
            rs.DeleteObject(crvs[i])
            crvs[i] = crv
    for i in range(len(crvs)):
        sections.append(splitCrv(crvs[i],[profile]))
    #The splitCrv creates lists of curves that are then checked to 
    #see if they lie within the profile (by looking at their midpoint)
    for i in range(len(sections)):
        for j in range(len(sections[i])):
            if rs.PointInPlanarClosedCurve(rs.CurveMidPoint(sections[i][j]),profile)==False:
                rs.DeleteObject(sections[i][j])
            hatch.append(sections[i][j])
    rs.DeleteObject(x)
    return crvs

##################################
# GENHATCHY function works the same way as the genHatchX
# it just uses the y axis of the bounding box as a reference
##################################

def genHatchY(profile,attPts,strength,ang,gap,min):
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
    for i in range(int(limit/gap)):
        crvs.append(rs.CopyObject(y,vecY*i*gap))
    for i in range(len(crvs)):
        relevant = []
        for j in range(len(attPts)):
            param = rs.CurveClosestPoint(crvs[i],attPts[j])
            close = rs.EvaluateCurve(crvs[i],param)
            if rs.Distance(attPts[j],close)<strength*2:
                relevant.append(attPts[j])
        if len(relevant)>0:
<<<<<<< HEAD
=======
            divPts = rs.DivideCurve(crvs[i],30)
>>>>>>> 59bfeb7955d50ab4cce208edcc2e618bf867223a
            for j in range(len(divPts)):
                sum = 0
                length = 0
                vecSum = [0,0,0]
                for k in range(len(relevant)):
                    attPt = relevant[k]
                    added = rs.Distance(attPt,divPts[j])/strength
                    if added<1:
                        vecSum = rs.PointAdd(vecSum,rs.VectorCreate(attPt,divPts[j]))
                        sum = sum+added
                        length = length+1
                if length!=0:
                    val = 1-sum/length
                    vec = rs.VectorScale(vecSum,val/length)
<<<<<<< HEAD
                    if j!=0 or j!=len(divPts)-1:
                        divPts[j]=rs.PointAdd(divPts[j],vec*(1-val))
                        if length>1:
                            print divPts
=======
                    if j!=0 and j!=len(divPts)-1:
                        divPts[j]=rs.PointAdd(divPts[j],vec*(1-val))
>>>>>>> 59bfeb7955d50ab4cce208edcc2e618bf867223a
            crv = rs.AddCurve(divPts)
            rs.DeleteObject(crvs[i])
            crvs[i] = crv
    for i in range(len(crvs)):
        sections.append(splitCrv(crvs[i],[profile]))
    for i in range(len(sections)):
        for j in range(len(sections[i])):
            if rs.PointInPlanarClosedCurve(rs.CurveMidPoint(sections[i][j]),profile)==False:
                rs.DeleteObject(sections[i][j])
            hatch.append(sections[i][j])
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
    attPts = rs.GetObjects("select points",rs.filter.point)
    angX = rs.GetReal("enter angle of hatch in first direction (degrees)", 30)
    angY = rs.GetReal("enter angle of hatch in second direction (degrees)", 40)
    spacingX = rs.GetReal("enter desired spacing of hatch in first direction",5)
    spacingY = rs.GetReal("enter desired spacing of hatch in second direction",6)
    strength = rs.GetReal("enter desired range for attractor",90)
    
    minX = rs.GetReal("enter minimum spacing in first direction",.1*spacingX)
    minY = rs.GetReal("enter minimum spacing in second direction",.1*spacingY)
    genHatchX(profile,attPts,strength,angX,spacingX,minX)
    genHatchY(profile,attPts,strength,angY,spacingY,minY)

Main()