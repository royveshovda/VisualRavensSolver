import numpy as np
import cv2
import math
import sys
# Your Agent for solving Raven's Progressive Matrices. You MUST modify this file.
#
# You may also create and submit new files in addition to modifying this file.
#
# Make sure your file retains methods with the signatures:
# def __init__(self)
# def Solve(self,problem)
#
# These methods will be necessary for the project's main method to run.
class Agent:
    # The default constructor for your Agent. Make sure to execute any
    # processing necessary before your Agent starts solving problems here.
    #
    # Do not add any variables to this signature; they will not be used by
    # main().
    def __init__(self):
        pass

    # The primary method for solving incoming Raven's Progressive Matrices.
    # For each problem, your Agent's Solve() method will be called. At the
    # conclusion of Solve(), your Agent should return a String representing its
    # answer to the question: "1", "2", "3", "4", "5", or "6". These Strings
    # are also the Names of the individual RavensFigures, obtained through
    # RavensFigure.getName().
    #
    # In addition to returning your answer at the end of the method, your Agent
    # may also call problem.checkAnswer(String givenAnswer). The parameter
    # passed to checkAnswer should be your Agent's current guess for the
    # problem; checkAnswer will return the correct answer to the problem. This
    # allows your Agent to check its answer. Note, however, that after your
    # agent has called checkAnswer, it will#not* be able to change its answer.
    # checkAnswer is used to allow your Agent to learn from its incorrect
    # answers; however, your Agent cannot change the answer to a question it
    # has already answered.
    #
    # If your Agent calls checkAnswer during execution of Solve, the answer it
    # returns will be ignored; otherwise, the answer returned at the end of
    # Solve will be taken as your Agent's answer to this problem.
    #
    # @param problem the RavensProblem your agent should solve
    # @return your Agent's answer to this problem

    def correspondingObject3x3LikelyOutput(self,objA,  attributesA, dictLikelyOutput):
        objectsB={}
        for i in dictLikelyOutput.keys():
            j=i.split(":")
            if(objectsB.has_key(j[0])):
                objectsB[j[0]]=objectsB[j[0]]+1
            else:
                objectsB[j[0]]=1

        if(len(objectsB)==1):
            for i in objectsB.keys():
                return i

        numAttrsObj = len(self.objGetAttributes(objA, attributesA))
        dictObjsB={}
        for i in objectsB.keys():
            numAttrsMatched=0
            for j in self.objGetAttributes(objA, attributesA):
                for k in dictLikelyOutput.keys():
                    l=k.split(":")
                    if(l[0]==i):
                        if(j==l[1] and attributesA[objA+":"+j]==dictLikelyOutput[k]):
                            numAttrsMatched = numAttrsMatched + 1
                            if i in dictObjsB.keys():
                                dictObjsB[i] = dictObjsB[i] + 1
                            else:
                                dictObjsB[i] = 1
                            if(numAttrsMatched==numAttrsObj):
                                return i
                            break
                        elif(j==l[1]):
                            if i in dictObjsB.keys():
                                dictObjsB[i] = dictObjsB[i] + 1
                            else:
                                dictObjsB[i] = 1
                            break
        maxMatched=-9999
        corObj=""
        diffInAttrs=1000
        for i in dictObjsB.keys():
            if (dictObjsB[i]>maxMatched or (dictObjsB[i]==maxMatched and ((objectsB[i]-numAttrsObj) < diffInAttrs))):
                maxMatched=dictObjsB[i]
                corObj=i
                diffInAttrs = objectsB[i]-numAttrsObj

        if(corObj!=""):
            return corObj

        return None

    def getObjects(self, dict):
        l = []
        for key in dict.keys():
            keySplit = key.split(":")
            if(not keySplit[0] in dict):
                l.append(keySplit[0])
        return l

    def objGetAttributes(self, obj, dict):
        l = []
        for key in dict.keys():
            keySplit = key.split(":")
            if(keySplit[0]==obj):
                l.append(keySplit[1])
        return l

    def correspondingObject3x3(self, objA, objectsA, objectsB, attributesA, attributesB):
        if(len(objectsB)==1):
            return objectsB[0]
        numAttrsObj = len(self.objGetAttributes(objA, attributesA))
        attrsMatched={}
        dictObjsB = {}
        for i in objectsB:
            attrsMatched[i]=0
            numAttrsMatched=0
            for j in self.objGetAttributes(objA, attributesA):
                for k in self.objGetAttributes(i,attributesB):
                    if(j==k and attributesA[objA+":"+j]==attributesB[i+":"+k]):
                        numAttrsMatched = numAttrsMatched + 1
                        attrsMatched[i]=numAttrsMatched
                        if i in dictObjsB.keys():
                            dictObjsB[i]=dictObjsB[i]+1
                        else:
                            dictObjsB[i]=1
                        if(numAttrsMatched == numAttrsObj):
                            return i
                        break
                    elif(j==k):
                        if i in dictObjsB.keys():
                            dictObjsB[i]=dictObjsB[i] + 1
                        else:
                            dictObjsB[i]=1
                        break

        print "Printing dictObjsB"
        for i in dictObjsB.keys():
            print i, dictObjsB[i],
        print
        maxSameAttributes=-9999
        maxAttrsMatched=-9999
        corObj=""
        diffInAttrs=1000
        for i in dictObjsB.keys():
            if (attrsMatched[i]> maxAttrsMatched or (attrsMatched[i]==maxAttrsMatched and dictObjsB[i]>maxSameAttributes) or (attrsMatched[i]==maxAttrsMatched and dictObjsB[i]==maxSameAttributes and ((len(self.objGetAttributes(i,attributesB))-numAttrsObj) < diffInAttrs))):
                maxAttrsMatched=attrsMatched[i]
                maxSameAttributes=dictObjsB[i]
                corObj=i
                diffInAttrs = len(self.objGetAttributes(i, attributesB))-numAttrsObj

        if(corObj!=""):
            return corObj

        return None
    def getSquareSizeIndex(self, size):
        if(size=="very-small"):
            return 0
        elif(size=="small"):
            return 1
        elif(size=="medium"):
            return 2
        elif(size=="large"):
            return 3
        elif(size=="very-large"):
            return 4
    def getRemSquareFilledList(self, l, objectsA, attributesA,objectsB,attributesB,objectsC,attributesC):
        nl = l
        for i in objectsA:
            if(attributesA[i+":shape"]=="square" and attributesA.has_key(i+":fill") and 
                    attributesA[i+":fill"]=="yes" 
                    and attributesA.has_key(i+":size")):
                nl[self.getSquareSizeIndex(attributesA[i+":size"])]=0
        for i in objectsB:
            if(attributesB[i+":shape"]=="square" and attributesB.has_key(i+":fill") and 
                    attributesB[i+":fill"]=="yes"
                    and attributesB.has_key(i+":size")):
                nl[self.getSquareSizeIndex(attributesB[i+":size"])]=0
        if(objectsC != None):
            for i in objectsC:
                if(attributesC[i+":shape"]=="square" and attributesC.has_key(i+":fill") and 
                        attributesC[i+":fill"]=="yes"
                        and attributesC.has_key(i+":size")):
                    nl[self.getSquareSizeIndex(attributesC[i+":size"])]=0

        return nl
    def getSquareFilledList(self, objectsA, attributesA):
        l=[0,0,0,0,0]
        for i in objectsA:
            if(attributesA[i+":shape"]=="square" and attributesA[i+":fill"]=="yes"):
                l[self.getSquareSizeIndex(attributesA[i+":size"])]=1
        return l

    def getNetRotations(self,objectsA, objectsB, attributesA, attributesB):
        count = 0
        for i in objectsA:
            for j in self.objGetAttributes(i,attributesA):
                if(j=="angle"):
                    if(attributesA[i+":"+j]==str(0)):
                        count = count + 1
                    elif(attributesA[i+":"+j]==str(180)):
                        count = count - 1
        for i in objectsB:
            for j in self.objGetAttributes(i,attributesB):
                if(j=="angle"):
                    if(attributesB[i+":"+j]==str(0)):
                        count = count + 1
                    elif(attributesB[i+":"+j]==str(180)):
                        count = count - 1
        return count
    def correspondingObject(self, objName, dictA, dictB):
        numAttrObjA = 0
        for key in dictA.keys():
            keySplit = key.split(":")
            if(keySplit[0]==objName):
                numAttrObjA = numAttrObjA + 1

        objectsBCount ={}
        
        for key in dictB.keys():
            keySplit = key.split(":")
            if (objectsBCount.has_key(keySplit[0])):
                objectsBCount[keySplit[0]] = objectsBCount[keySplit[0]] + 1
            else:
                objectsBCount[keySplit[0]] = 1

        for key in objectsBCount.keys():
            if (objectsBCount[key] == numAttrObjA):
                return key
       
        for key in objectsBCount.keys():
            if(dictA.has_key(objName+":shape") and dictB.has_key(key+":shape") and dictA[objName+":shape"] == dictB[key+":shape"]):
                if(dictA.has_key(objName+":fill") and dictB.has_key(key+":fill") and dictA[objName+":fill"] == dictB[key+":fill"]):
                    return key
        return None

    def numObjects(self, dict):
        numObjects = 0
        objects = []
        for key in dict.keys():
            keySplit = key.split(":")
            if((not keySplit[0] in objects) and keySplit[0] != "likeliness"):
                objects.append(keySplit[0])
                numObjects = numObjects + 1

        return numObjects

    def getObjectFromName(self, name, objects):
        for i in objects:
            if(i.getName()==name):
                return i

    def alreadyAddedObject(self, attributes, objects, numEdges, area, cx, cy):
        for object in objects:
            if(numEdges <= 8):
                if(attributes[object+":numEdges"]==numEdges and abs(attributes[object+":cx"] - cx) < 5 and 
                        abs(attributes[object+":cy"] - cy) < 5 and
                        abs(attributes[object+":area"] - area) < 2000):
                    return object
            else:
                if(abs(attributes[object+":cx"]-cx)<5 and abs(attributes[object+":cy"]-cy)<5 and
                  abs(attributes[object+":area"] - area) < 2000):
                    return object
        return None
    def filterDict(self, dict):
        keys = dict.keys()
        for key in keys:
            keySplit = key.split(":")
            if(keySplit[1] == "area" or keySplit[1]=="cx" or keySplit[1]=="cy" or keySplit[1]=="numEdges"):
                del dict[key]
        return dict

    def pacManGetAngle(self, approx, cx, cy):
        closestToCenter = -1
        minDistance = sys.float_info.max

        for i in range(0, len(approx)):
            curDistance = math.sqrt(math.pow(approx[i][0][0]-cx,2)+math.pow(approx[i][0][1]-cy,2))
            if(curDistance < minDistance):
                closestToCenter = i
                minDistance = curDistance
        next = (closestToCenter + 1)% len(approx)
        prev = (closestToCenter - 1) % len(approx)

        p0x = approx[closestToCenter][0][0]
        p0y = approx[closestToCenter][0][1]

        p1x = float(approx[next][0][0] + approx[prev][0][0])/2
        p1y = float(approx[next][0][1] + approx[prev][0][1])/2

        angle = cv2.fastAtan2(p1y-p0y,p1x-p0x)
        print "pac man angle is " + str(angle)
        if(abs(angle)<10 or abs(angle-360)<10):
            angle = 0
        elif(abs(angle-45)<10):
            angle = 45
        elif(abs(angle-90)<10):
            angle = 90
        elif(abs(angle-135)<10):
            angle = 135
        elif(abs(angle-180)<10):
            angle = 180
        elif(abs(angle-225)<10):
            angle = 225
        elif(abs(angle-270)<10):
            angle = 270
        elif(abs(angle-315)<10):
            angle = 315
        return angle

    def getTriangleSpecifics(self, approx):
        length = len(approx)
        isRightTriangle = False
        vertex = -1
        print "approx"
        print approx
        for i in range(0,length):
            Ax = approx[i][0][0]
            Ay = approx[i][0][1]

            Bx = approx[(i+1)%3][0][0]
            By = approx[(i+1)%3][0][1]

            Cx = approx[(i+2)%3][0][0]
            Cy = approx[(i+2)%3][0][1]

            v0x = Ax - Bx
            v0y = Ay - By

            v1x = Cx - Bx
            v1y = Cy - By

            angle1 = cv2.fastAtan2(v0y,v0x)
            angle2 = cv2.fastAtan2(v1y,v1x)

            if(abs(abs(angle1-angle2)-90)<10 or abs(abs(angle1-angle2)-270)<10):
                isRightTriangle = True
                vertex = (i+1)%3
                break

        if(isRightTriangle):
            print "vertex is "
            print approx[vertex][0]

            midpointx = (approx[(vertex+1)%3][0][0] + approx[(vertex+2)%3][0][0])/2
            midpointy = (approx[(vertex+1)%3][0][1] + approx[(vertex+2)%3][0][1])/2

            v1x = approx[vertex][0][0] - midpointx
            v1y = approx[vertex][0][1] - midpointy

            angle = cv2.fastAtan2(v1y,v1x)
            angle = angle + 225
            if(angle>360):
                angle = angle - 360
            print "angle is " + str(angle)
            if(abs(angle)<10 or abs(angle-360)<10):
                angle = 0
            elif(abs(angle-45)<10):
                angle = 45
            elif(abs(angle-90)<10):
                angle = 90
            elif(abs(angle-135)<10):
                angle = 135
            elif(abs(angle-180)<10):
                angle = 180
            elif(abs(angle-225)<10):
                angle = 225
            elif(abs(angle-270)<10):
                angle = 270
            elif(abs(angle-315)<10):
                angle = 315

            return ("right-triangle", angle)
        else:
            vertex = -1
            angle = 0
            for i in range(0, length):
                p0x = approx[i][0][0]
                p0y = approx[i][0][1]

                p1x = approx[(i+1)%3][0][0]
                p1y = approx[(i+1)%3][0][1]

                if(abs(p0x-p1x)<10 or abs(p0y-p1y)<10):
                    vertex = (i+2)%3
                    break
            if(vertex!= -1):
                midPointOppX = (approx[(vertex+1)%3][0][0] + approx[(vertex+2)%3][0][0])/2
                midPointOppY = (approx[(vertex+1)%3][0][1] + approx[(vertex+2)%3][0][1])/2

                vX = approx[vertex][0][0] - midPointOppX
                vY = approx[vertex][0][1] - midPointOppY

                angle = cv2.fastAtan2(vY,vX)
                angle = angle + 90
                if(angle > 360):
                    angle = angle -360
                if(abs(angle)<10 or abs(angle-360)<10):
                    angle = 0
                elif(abs(angle-45)<10):
                    angle = 45
                elif(abs(angle-90)<10):
                    angle = 90
                elif(abs(angle-135)<10):
                    angle = 135
                elif(abs(angle-180)<10):
                    angle = 180
                elif(abs(angle-225)<10):
                    angle = 225
                elif(abs(angle-270)<10):
                    angle = 270
                elif(abs(angle-315)<10):
                    angle = 315

                print "angle is" + str(angle)
            return ("triangle", angle)


    def getShape(self, approx, cx, cy, theta):
        length = len(approx)

        # check if plus
        isPlus = True
        for i in range(0,length):
            px0 = approx[i][0][0]
            py0 = approx[i][0][1]

            px1 = approx[(i+1)%length][0][0]
            py1 = approx[(i+1)%length][0][1]
            angle = cv2.fastAtan2(py1 - py0, px1 - px0)
            if (abs(angle-(0+abs(theta)))> 10 and abs(angle-(90+abs(theta))) > 10 
                    and abs(angle-(180+abs(theta))) > 10 and abs(angle-(270+abs(theta))) > 10 and abs(angle-(360+abs(theta)))>10):
                isPlus = False
        if(isPlus):
            return "plus"

        # check if circle
        isCircle = True
        distance0 = math.sqrt(math.pow(approx[i][0][0]-cx,2)+math.pow(approx[i][0][1]-cy,2))
        print "radius = " + str(distance0)
        for i in range(1,length):
            px0 = approx[i][0][0]
            py0 = approx[i][0][1]
            distance = math.sqrt(math.pow(approx[i][0][0] - cx,2) + math.pow(approx[i][0][1]-cy, 2))
            if (abs(distance-distance0)>= 5):
                isCircle = False
                break
        if(isCircle):
            return "circle"
        # if its neither plus nor circle, then its pac-man
        return "Pac-Man"

    def Solve(self,problem):
        attributesA = {}
        attributesB = {}
        attributesC = {}
        # Below dictionaries are only needed for 3x3 matrices
        attributesD = {}
        attributesE = {}
        attributesF = {}
        attributesG = {}
        attributesH = {}
        attributesOfLikelyOutput = {}

        objectsA = []
        objectsB = []
        objectsC = []
        objectsD = []
        objectsE = []
        objectsF = []
        objectsG = []
        objectsH = []

        problemType = problem.getProblemType()

        print problem.getName()

        figures = problem.getFigures()
        objName = 0
        for key in figures.keys():
            attributes = {}
            contourObjects = {}
            objects = []
            imgName = figures[key].getName()
            print imgName
            imgPath = figures[key].getPath()
            img = cv2.imread(imgPath)
            gray = cv2.imread(imgPath, 0)
            ret,thresh = cv2.threshold(gray, 127, 255, 1)
            contours, hierarchy = cv2.findContours(thresh, 3, 2)
            for i in range (0, len(contours), 1):

                approx = cv2.approxPolyDP(contours[i],0.01*cv2.arcLength(contours[i],True),True)
                area = cv2.contourArea(contours[i])
                M = cv2.moments(contours[i])
                rect = cv2.minAreaRect(contours[i])
                boundRectArea = rect[1][0] * rect[1][1]
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                dup = self.alreadyAddedObject(attributes, objects, len(approx), area, cx, cy)
                if(dup == None):
                    # numEdges (shape)
                    print "approx"
                    print approx
                    attributes[str(objName)+":numEdges"] = len(approx)
                    if(len(approx) > 8):
                        print "rect"
                        print rect
                        shape = self.getShape(approx, cx, cy, int(rect[2])+45)
                        attributes[str(objName)+":shape"] = shape
                        if(shape=="Pac-Man"):
                            attributes[str(objName)+":angle"] = str(self.pacManGetAngle(approx, cx, cy))
                        elif(shape=="circle"):
                            attributes[str(objName)+":angle"] = str(0)
                        elif(shape=="plus"):
                            attributes[str(objName)+":angle"] = str(int(rect[2])+45)
                    elif(len(approx) == 3):
                        shape,angle = self.getTriangleSpecifics(approx)
                        attributes[str(objName)+":shape"] = shape
                        attributes[str(objName)+":angle"] = str(angle)
                    elif(len(approx) == 4):
                        attributes[str(objName)+":shape"] = "square"
                        attributes[str(objName)+":angle"] = str(int(abs(rect[2])))
                    elif(len(approx) == 8):
                        attributes[str(objName)+":shape"] = "octagon"
                    else:
                        attributes[str(objName)+":shape"] = "unknown"

                    # area
                    attributes[str(objName)+":area"] = area

                    #size
                    print "boundRectArea is " + str(boundRectArea)
                    if(boundRectArea < 4500):
                        attributes[str(objName)+":size"] = "very-small"
                    elif(boundRectArea < 7500):
                        attributes[str(objName)+":size"] = "small"
                    elif(boundRectArea < 14000):
                        attributes[str(objName)+":size"] = "medium"
                    elif(boundRectArea < 21000):
                        attributes[str(objName)+":size"] = "large"
                    else:
                        attributes[str(objName)+":size"] = "very-large"
                    
                    # centroid
                    attributes[str(objName)+":cx"] = cx
                    attributes[str(objName)+":cy"] = cy

                    # find fill of color
                    mask = np.zeros(gray.shape, np.uint8)
                    cv2.drawContours(mask, contours, i, 255, -1)
                    if(hierarchy[0][i][2] != -1):
                        maskChild = np.zeros(gray.shape, np.uint8)
                        cv2.drawContours(maskChild, contours, hierarchy[0][i][2],255, -1)
                        mask = mask - maskChild
                    if(cv2.mean(gray,mask)[0] < 20):
                        attributes[str(objName)+":fill"] = 'yes'
                    else:
                        attributes[str(objName)+":fill"] = 'no'

                    # inside
                    if(hierarchy[0][i][3] != -1):
                        attributes[str(objName)+":inside"] = contourObjects[hierarchy[0][i][3]] 

                    objects.append(str(objName))
                    contourObjects[i] = str(objName)
                    objName = objName + 1
                else:
                    contourObjects[i] = dup

                    #find fill color
                    mask = np.zeros(gray.shape, np.uint8)
                    cv2.drawContours(mask, contours, i, 255, -1)
                    if(hierarchy[0][i][2] != -1):
                        maskChild = np.zeros(gray.shape, np.uint8)
                        cv2.drawContours(maskChild, contours, hierarchy[0][i][2],255, -1)
                        mask = mask - maskChild
                    if(cv2.mean(gray,mask)[0] < 20):
                        attributes[dup+":fill"] = 'yes'
                    else:
                        attributes[dup+":fill"] = 'no'
            if(imgName == "A"):
                attributesA = attributes
                objectsA = objects
            elif(imgName == "B"):
                attributesB = attributes
                objectsB = objects
            elif(imgName == "C"):
                attributesC = attributes
                objectsC = objects
            elif(imgName == "D"):
                attributesD = attributes
                objectsD = objects
            elif(imgName == "E"):
                attributesE = attributes
                objectsE = objects
            elif(imgName == "F"):
                attributesF = attributes
                objectsF = objects
            elif(imgName == "G"):
                attributesG = attributes
                objectsG = objects
            elif(imgName == "H"):
                attributesH = attributes
                objectsH = objects
            elif(imgName == "1"):
                attributes1 = attributes
                objects1 = objects
            elif(imgName == "2"):
                attributes2 = attributes
                objects2 = objects
            elif(imgName == "3"):
                attributes3 = attributes
                objects3 = objects
            elif(imgName == "4"):
                attributes4 = attributes
                objects4 = objects
            elif(imgName == "5"):
                attributes5 = attributes
                objects5 = objects
            elif(imgName == "6"):
                attributes6 = attributes
                objects6 = objects

        numObjectsA = len(objectsA)
        numObjectsB = len(objectsB)
        numObjectsC = len(objectsC)
        numObjectsD = len(objectsD)
        numObjectsE = len(objectsE)
        numObjectsF = len(objectsF)
        numObjectsG = len(objectsG)
        numObjectsH = len(objectsH)


        attributesA = self.filterDict(attributesA)
        attributesB = self.filterDict(attributesB)
        attributesC = self.filterDict(attributesC)
        attributesD = self.filterDict(attributesD)
        attributesE = self.filterDict(attributesE)
        attributesF = self.filterDict(attributesF)
        attributesG = self.filterDict(attributesG)
        attributesH = self.filterDict(attributesH)
        
        attributesOutputList = []
        objectsOutputList = []
        numAttrsMatched = []

        attributesOutputList.append(attributes1)
        attributesOutputList.append(attributes2)
        attributesOutputList.append(attributes3)
        attributesOutputList.append(attributes4)
        attributesOutputList.append(attributes5)
        attributesOutputList.append(attributes6)
        objectsOutputList.append(objects1)
        objectsOutputList.append(objects2)
        objectsOutputList.append(objects3)
        objectsOutputList.append(objects4)
        objectsOutputList.append(objects5)
        objectsOutputList.append(objects6)

        for i in range(0,6):
            attributesOutputList[i] = self.filterDict(attributesOutputList[i])
            attributesOutputList[i]['likeliness'] = 0
            numAttrsMatched.append(0);

        print "Printing dict A"
        print attributesA
        print "Printing dict B"
        print attributesB
        print "Printing dict C"
        print attributesC
        if(problemType == "3x3 (Image)"):
            print "Printing dict D"
            print attributesD
            print "Printing dict E"
            print attributesE
            print "Printing dict F"
            print attributesF
            print "Printing dict G"
            print attributesG
            print "Printing dict H"
            print attributesH

        sizes = ['very-small', 'small', 'medium', 'large', 'very-large']

        if(problemType == "3x3 (Image)"):
            l1 = [1,1,1,1,1]
            l1 = self.getRemSquareFilledList(l1, objectsA,attributesA, objectsB, attributesB, objectsC, attributesC)
            print "Printing L1"
            print l1
            l2 = [1,1,1,1,1]
            l2 = self.getRemSquareFilledList(l2, objectsD,attributesD, objectsE, attributesE, objectsF, attributesF)
            print "Printing L2"
            print l2
                
            if(l1[0]==0 and l1[1]==0 and l1[2]==0 and l1[3]==0 and l1[4]==0
                    and l2[0]==0 and l2[1]==0 and l2[2]==0 and l2[3]==0 and l2[4]==0):
                l3 = [1,1,1,1,1]
                l3 = self.getRemSquareFilledList(l3, objectsG,attributesG, objectsH, attributesH, None, None)
                print "Printing L3"
                print l3
                for i in range(1,7):
                    l4=self.getSquareFilledList(objectsOutputList[i-1],attributesOutputList[i-1])
                    if(l4==l3):
                        print "Output is " + str(i)
                        return str(i)
            if(numObjectsA==numObjectsB==numObjectsC==numObjectsD==numObjectsE==numObjectsF==numObjectsG==numObjectsH):
                for k in objectsA:
                    objNameA = k
                    print "objName = " + objNameA
                    corObjectAB = self.correspondingObject3x3(k, objectsA, objectsB, attributesA, attributesB)
                    corObjectAC = self.correspondingObject3x3(k, objectsA, objectsC, attributesA, attributesC)
                    corObjectAG = self.correspondingObject3x3(k, objectsA, objectsG, attributesA, attributesG)
                    corObjectAH = self.correspondingObject3x3(k, objectsA, objectsH, attributesA, attributesH)
                    corObjectAD = self.correspondingObject3x3(k, objectsA, objectsD, attributesA, attributesD)
                    corObjectAE = self.correspondingObject3x3(k, objectsA, objectsE, attributesA, attributesE)
                    corObjectAF = self.correspondingObject3x3(k, objectsA, objectsF, attributesA, attributesF)
                    print "objName = " + objNameA + ",corAB = " + corObjectAB + ",corAC = " + corObjectAC + ",corAD="+corObjectAD +\
                            ",corAE=" + corObjectAE+ ",corAF="+corObjectAF+",corAG="+corObjectAG+",corAH="+corObjectAH

                    for i in attributesA.keys():
                        print "Printing key " + i
                        keySplit = i.split(":")
                        if(keySplit[0] != k):
                            continue
                        attrName = keySplit[1]
                        unmatchingAttrVals = []
                        if(attributesB.has_key(corObjectAB+":"+attrName) and attributesC.has_key(corObjectAC+":"+attrName) and attributesA[objNameA+":"+attrName]==attributesB[corObjectAB+":"+attrName]==attributesC[corObjectAC+":"+attrName]):
                                if(attributesG[corObjectAG+":"+attrName]==attributesH[corObjectAH+":"+attrName]):
                                    attributesOfLikelyOutput[corObjectAC+":"+attrName] = attributesG[corObjectAG+":"+attrName]
                        else:
                             unmatchingAttrVals.append(attributesA[objNameA+":"+attrName])
                             if(attributesB.has_key(corObjectAB+":"+attrName)):
                                unmatchingAttrVals.append(attributesB[corObjectAB+":"+attrName])
                             if(attributesC.has_key(corObjectAC+":"+attrName)):
                                unmatchingAttrVals.append(attributesC[corObjectAC+":"+attrName])
                             if(attributesD.has_key(corObjectAD+":"+attrName) and 
                                     attributesD[corObjectAD+":"+attrName] in unmatchingAttrVals 
                               and attributesE.has_key(corObjectAE+":"+attrName) and 
                                   attributesE[corObjectAE+":"+attrName] in unmatchingAttrVals
                               and attributesF[corObjectAF+":"+attrName] in unmatchingAttrVals):
                                    
                                if(attributesG[corObjectAG+":"+attrName] in unmatchingAttrVals):
                                    unmatchingAttrVals.remove(attributesG[corObjectAG+":"+attrName])
                                if(attributesH[corObjectAH+":"+attrName] in unmatchingAttrVals):
                                    unmatchingAttrVals.remove(attributesH[corObjectAH+":"+attrName])
                                if(len(unmatchingAttrVals)==1):
                                    attributesOfLikelyOutput[corObjectAC+":"+attrName]=unmatchingAttrVals[0]
                             elif(attrName=="size"):
                                if(sizes.index(attributesB[corObjectAB+":"+attrName])>sizes.index(attributesA[objNameA+":"+attrName])
                                  and sizes.index(attributesC[corObjectAC+":"+attrName])>sizes.index(attributesB[corObjectAB+":"+attrName])):
                                  if(sizes.index(attributesH[corObjectAH+":"+attrName])>sizes.index(attributesG[corObjectAG+":"+attrName])):
                                      attributesOfLikelyOutput[corObjectAC+":"+attrName]=sizes[1+sizes.index(attributesH[corObjectAH+":"+attrName])]
                             elif(attrName=="angle"):
                                 if(attributesB.has_key(corObjectAB+":"+attrName) and
                                         attributesH.has_key(corObjectAH+":"+attrName) and
                                         int(attributesB[corObjectAB+":"+attrName])-int(attributesA[objNameA+":"+attrName]) == 
                                         int(attributesH[corObjectAH+":"+attrName])-int(attributesG[corObjectAG+":"+attrName])):
                                     attributesOfLikelyOutput[corObjectAC+":"+attrName]=str((int(attributesH[corObjectAH+":"+attrName]) + \
                                         int(attributesC[corObjectAC+":"+attrName])-int(attributesB[corObjectAB+":"+attrName]))%360)
                                 else:
                                     if(attributesC.has_key(corObjectAC+":"+attrName) and
                                             attributesB.has_key(corObjectAB+":"+attrName) and
                                             (int(attributesC[corObjectAC+":"+attrName]) == 
                                             int(attributesA[objNameA+":"+attrName])+int(attributesB[corObjectAB+":"+attrName]))):
                                         attributesOfLikelyOutput[corObjectAC+":"+attrName]=\
                                          str((int(attributesG[corObjectAG+":"+attrName]) + int(attributesH[corObjectAH+":"+attrName]))%360)
                             elif(attrName=="inside" and attributesA[objNameA+":inside"]!= ""):
                                    print objNameA
                                    o = attributesA[objNameA+":inside"] 
                                    corB = self.correspondingObject3x3(o, objectsA, objectsB,attributesA, attributesB)
                                    corC = self.correspondingObject3x3(o, objectsA, objectsC, attributesA, attributesC)
                                    if(corB==attributesB[corObjectAB+":inside"] and corC==attributesC[corObjectAC+":inside"]):
                                        attributesOfLikelyOutput[corObjectAC+":inside"]=attributesC[corObjectAC+":inside"]
            elif((numObjectsB-numObjectsA)!=0 and numObjectsB-numObjectsA == numObjectsC-numObjectsB == numObjectsE-numObjectsD 
                         == numObjectsF-numObjectsE == numObjectsH-numObjectsG):
                numObjectsOutput = numObjectsH + (numObjectsB-numObjectsA)
                print "numObjectsOutput = " + str(numObjectsOutput)
                for i in objectsA:
                   corObjectAB = self.correspondingObject3x3(i,objectsA,objectsB, attributesA, attributesB)
                   corObjectAC = self.correspondingObject3x3(i,objectsA,objectsC, attributesA, attributesC)
                   corObjectAD = self.correspondingObject3x3(i,objectsA,objectsD, attributesA, attributesD)
                   corObjectAG = self.correspondingObject3x3(i,objectsA,objectsG, attributesA, attributesG)
                   for j in self.objGetAttributes(i, attributesA):
                       for k in range(0,numObjectsOutput):
                            if(attributesA[i+":"+j]==attributesB[corObjectAB+":"+j]
                                    ==attributesC[corObjectAC+":"+j]):
                                attributesOfLikelyOutput[str(k)+":"+j] = attributesG[corObjectAG+":"+j]
                            elif(attributesA[i+":"+j]==attributesD[corObjectAD+":"+j]
                                    ==attributesG[corObjectAG+":"+j]):
                                attributesOfLikelyOutput[str(k)+":"+j.getName()] = attributesC[corObjectAC+":"+j]
                            elif(j.getName()=="angle"):
                                theta=int(attributesC[corObjectAC+":"+j]) - int(attributesA[i.getName()+":"+j])
                                attributesOfLikelyOutput[str(k)+":"+j] = str(int(attributesG[corObjectAG+":"+j]) + theta)
            elif(numObjectsB==2*numObjectsA and numObjectsC==3*numObjectsA and
                    numObjectsE==2*numObjectsD and numObjectsF==3*numObjectsD and numObjectsH==2*numObjectsG):
                 numObjectsOutput=3*numObjectsG
                 for i in objectsA:
                    corObjectAB = self.correspondingObject3x3(i,objectsA,objectsB, attributesA, attributesB)
                    corObjectAC = self.correspondingObject3x3(i,objectsA,objectsC, attributesA, attributesC)
                    corObjectAD = self.correspondingObject3x3(i,objectsA,objectsD, attributesA, attributesD)
                    corObjectAG = self.correspondingObject3x3(i,objectsA,objectsG, attributesA, attributesG)
                    for j in self.objGetAttributes(i,attributesA):
                        for k in range(0,numObjectsOutput):
                            if(attributesB.has_key(corObjectAB+":"+j) and 
                                    attributesA[i+":"+j]==attributesB[corObjectAB+":"+j]
                                    ==attributesC[corObjectAC+":"+j]):
                                attributesOfLikelyOutput[str(k)+":"+j] = attributesG[corObjectAG+":"+j]
                            elif(attributesC.has_key(corObjectAC+":"+j) and 
                                    attributesA[i+":"+j]==attributesD[corObjectAD+":"+j]
                                    ==attributesG[corObjectAG+":"+j]):
                                attributesOfLikelyOutput[str(k)+":"+j] = attributesC[corObjectAC+":"+j]
                            elif(j=="angle" and attributesC.has_key(corObjectAC+":"+j)):
                                theta=int(attributesC[corObjectAC+":"+j]) - int(attributesA[i+":"+j])
                                attributesOfLikelyOutput[str(k)+":"+j] = str(int(attributesG[corObjectAG+":"+j]) + theta)
            else:
               
                print "Computing diff1"
                diff1 = self.getNetRotations(objectsA,objectsB,attributesA,attributesB)
                print "Computing diff2"
                diff2 = self.getNetRotations(objectsD,objectsE,attributesD,attributesE)
                print "diff1 = " + str(diff1) + "diff2 = " + str(diff2)
                if(abs(diff1)==len(objectsC) and abs(diff2)==len(objectsF)):
                    numObjectsOutput = self.getNetRotations(objectsG,objectsH,attributesG,attributesH)
                    for i in range(1,2):
                        if(len(objectsOutputList[i-1])==abs(numObjectsOutput)):
                            count=0
                            for j in objectsOutputList[i-1]:
                                for k in self.objGetAttributes(j,attributesOutputList[i-1]):
                                    if(k=="angle"):
                                        if(numObjectsOutput<0 and attributesOutputList[i-1][j+":"+k]==str(180)):
                                            count = count + 1
                                        elif(numObjectsOutput>0 and attributesOutputList[i-1][j+":"+k]==str(0)):
                                            count = count + 1
                            if(count==abs(numObjectsOutput)):
                                    return str(i)
                if(not attributesOfLikelyOutput and len(objectsC) == len(objectsF)):
                    for i in objectsC:
                        corCF = self.correspondingObject3x3(i, objectsC, objectsF, attributesC, attributesF)
                        corCG = self.correspondingObject3x3(i, objectsC, objectsG, attributesC, attributesG)
                        print "objName = " + i + " corCF = " + corCF + " corCG = " + corCG
                        for attr in self.objGetAttributes(i, attributesC):
                            if(attr != "inside"):
                                if(not (attr == "angle" and attributesG[corCG+":shape"] == "octagon")):
                                    attributesOfLikelyOutput[i+":"+attr] = attributesG[corCG+":"+attr]
                            else:
                                attributesOfLikelyOutput[i+":"+attr] = attributesC[i+":"+attr]

        # start 2x1 and 2x2
        else:
            for i in attributesA.keys():
                if i in attributesB:
                    if (attributesA[i] == attributesB[i]):
                        if i in attributesC:
                            attributesOfLikelyOutput[i] = attributesC[i]
                    else:
                        if i in attributesC:
                            if (attributesA[i] == attributesC[i]):
                                attributesOfLikelyOutput[i] = attributesB[i]
                            else:
                                attribute = i.split(':')
                                if (len(attribute)>1 and attribute[1] == 'angle'):
                                    attributesOfLikelyOutput[i] = str((int(attributesC[i]) + (int(attributesB[i]) - int(attributesA[i]))) % 360)
                                elif(len(attribute)>1 and attribute[1] == 'shape'):
                                    for attr in attributesA.keys():
                                        if(attributesA[attr] == attributesB[i]):
                                            transformedTo = attr.split(":")
                                            if(attributesC.has_key(transformedTo[0]+":shape")):
                                                attributesOfLikelyOutput[i] = attributesC[transformedTo[0] + ':shape'] 
                else:
                    j = i.split(":")
                    objName = j[0]
                    corObjectAB = self.correspondingObject(j[0], attributesA, attributesB)
                    corObjectAC = self.correspondingObject(j[0], attributesA, attributesC)

                    if(corObjectAB != None and corObjectAC != None):
                        if(attributesC.has_key(corObjectAC+":"+j[1]) and attributesB.has_key(corObjectAB+":"+j[1]) and attributesA[objName+":"+j[1]] == attributesB[corObjectAB+":"+j[1]]):
                            attributesOfLikelyOutput[corObjectAC+":"+j[1]]= attributesC[corObjectAC+":"+j[1]]
                        elif(attributesB.has_key(corObjectAB +":"+j[1]) and attributesC.has_key(corObjectAC+":"+j[1]) and attributesA[objName+":"+j[1]] == attributesC[corObjectAC+":"+j[1]]):
                            attributesOfLikelyOutput[corObjectAC+":"+j[1]]= attributesB[corObjectAB+":"+j[1]]
                        elif (j[1] == "angle"):
                         # reflection checks
                            if(attributesC[corObjectAC+":shape"]== "Pac-Man"):
                                angleA = int(attributesA[i])
                                angleB = int(attributesB[corObjectAB+":"+j[1]])
                                angleC = int(attributesC[corObjectAC+":"+j[1]])
                                if((angleA >= 0 and angleA <= 180 and angleB == 180 - angleA) or 
                                    (angleA >=180 and angleA <=360 and angleB == 540 - angleA)):
                                    if(angleC >=0 and angleC <=180):
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(180 - angleC)
                                    elif(angleC >=180 and angleC <=360):
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(540 - angleC)
                            elif(attributesC[corObjectAC+":shape"] == "right-triangle"):
                                angleA = int(attributesA[i])
                                angleB = int(attributesB[corObjectAB+":"+j[1]])
                                angleC = int(attributesC[corObjectAC+":"+j[1]])
                                if((angleA >=0 and angleA<=45 and angleB == 270 - 2*angleA) or (angleA>=45 and angleA<=90 and angleB == 180+angleA)
                                    or (angleA>=90 and angleA<=270 and angleB == 270-angleA) 
                                    or (angleA>=270 and angleA<=360 and angleB==(630-angleA)%360)):
                                    if(angleC>=0 and angleC<=45):
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(270 - 2*angleC)
                                    elif(angleC>=45 and angleC<=90):
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(180 + angleC)
                                    elif(angleC>=90 and angleC<=270):
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(270 - angleC)
                                    elif(angleC>=270 and angleC<=360):
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str((630 - angleC)%360)
                                else: # rotation
                                    delta = int(attributesB[corObjectAB+":"+j[1]]) - int(attributesA[i])
                                    newAngle = int(attributesC[corObjectAC+":"+j[1]]) + delta
                                    attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(newAngle % 360)
                            # rotation checks for other figures
                            elif(attributesC.has_key(corObjectAC+":"+j[1]) and attributesC[corObjectAC+":shape"] == "circle"):
                            # no change for circle rotation
                                attributesOfLikelyOutput[corObjectAC+":"+j[1]]=attributesC[corObjectAC+":"+j[1]]
                            elif(attributesB.has_key(corObjectAB+":"+j[1]) and attributesC.has_key(corObjectAC+":"+j[1])):
                                delta = int(attributesB[corObjectAB+":"+j[1]]) - int(attributesA[i])
                                # reflection check around Y-axis
                                if(int(attributesB[corObjectAB+":"+j[1]]) == 360 - int(attributesA[i])):
                                    attributesOfLikelyOutput[corObjectAC+":"+j[1]] = \
                                    str((360 - int(attributesC[corObjectAC+":"+j[1]]))%360)
                                else:
                                    newAngle = int(attributesC[corObjectAC+":"+j[1]]) + delta
                                    if(attributesC[corObjectAC+":shape"] == "square"):
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(newAngle % 90)
                                    else:
                                        attributesOfLikelyOutput[corObjectAC+":"+j[1]] = str(newAngle % 360)
                        elif (j[1] == "overlaps" or j[1] == "inside"):
                             if(attributesB.has_key(corObjectAB+":"+j[1]) and
                                     attributesC.has_key(corObjectAC+":"+j[1]) and
                                     self.correspondingObject(attributesA[i], attributesA, attributesB) == attributesB[corObjectAB+":"+j[1]]):
                                attributesOfLikelyOutput[corObjectAC+":"+j[1]] =attributesC[corObjectAC+":"+j[1]]
                         
        # if objects added in B from A, add them to D
        if(problemType=="2x1 (Image)" or problemType == "2x2 (Image)"):
            if(self.numObjects(attributesB) > self.numObjects(attributesA)):
                for obj in objectsB:
                    if(self.correspondingObject(obj, attributesB, attributesA) == None):
                        for key in attributesB.keys():
                            keySplit = key.split(":")
                            if(keySplit[0] == obj):
                                if(keySplit[1] == "inside"):
                                    attributesOfLikelyOutput[obj+":"+keySplit[1]] = self.correspondingObject(self.correspondingObject(attributesB[key], attributesB, attributesA), attributesA, attributesC)
                                else:
                                    attributesOfLikelyOutput[obj+":"+keySplit[1]] = attributesB[key]
        
        # if no objects in A, then it means from A to B it is pure addition. Hence whole of C goes into output as well
                if (self.numObjects(attributesA) == 0):
                    for key in attributesC.keys():
                        attributesOfLikelyOutput[key] = attributesC[key]
        
        # when objects added from A to C
        if(problemType=="2x1 (Image)" or problemType == "2x2 (Image)"):
            if(self.numObjects(attributesC) > self.numObjects(attributesA)):
                for obj in objectsC:
                    exists = 0
                    for key1 in attributesOfLikelyOutput.keys():
                        keySplit1 = key1.split(":")
                        if(obj == keySplit1[0]):
                            exists = 1
                            break
                
                    if(exists == 0):
                        corObjectA = self.correspondingObject(obj, attributesC, attributesA)
                        if(corObjectA != None):
                            corObjectB = self.correspondingObject(corObjectA, attributesA, attributesB)
                    
                        if(corObjectA != None and corObjectB != None):
                            for key2 in attributesC.keys():
                                keySplit2 = key2.split(":")
                                if(keySplit2[0] == obj):
                                    if(attributesA.has_key(corObjectA+":"+keySplit2[1]) and attributesC[key2] == attributesA[corObjectA+":"+keySplit2[1]]):
                                        attributesOfLikelyOutput[key2] = attributesB[corObjectB+":"+keySplit2[1]]


        print "Printing attributesOfLikelyOutput"
        print attributesOfLikelyOutput

        for i in range(0,6):
            outputI = attributesOutputList[i]
            for attrOut in outputI.keys():
                attrOutSplit = attrOut.split(":")
                objName = attrOutSplit[0]
                if(objName != 'likeliness'):
                    if(problemType=="3x3 (Image)"):
                        corObjectOut = self.correspondingObject3x3LikelyOutput(objName, attributesOutputList[i],attributesOfLikelyOutput);
                    else:
                        corObjectOut = self.correspondingObject(objName, attributesOutputList[i], attributesOfLikelyOutput)
                    if(corObjectOut != None and attrOutSplit[1] and attributesOfLikelyOutput.has_key(corObjectOut+":"+attrOutSplit[1])):
                        if(attrOutSplit[1] == "inside"):
                            if(self.correspondingObject(outputI[attrOut], attributesOutputList[i], attributesOfLikelyOutput) 
                                    == attributesOfLikelyOutput[corObjectOut+":"+attrOutSplit[1]]):
                                outputI['likeliness'] = outputI['likeliness'] + 1
                                numAttrsMatched[i] = numAttrsMatched[i] + 1

                        else:    
                            if (outputI[attrOut] == attributesOfLikelyOutput[corObjectOut+":"+attrOutSplit[1]]):
                                numAttrsMatched[i] = numAttrsMatched[i] + 1
                                if(attrOutSplit[1] == "shape"):
                                    outputI['likeliness'] = outputI['likeliness'] + 5
                                elif(attrOutSplit[1] == "fill"):
                                    outputI['likeliness'] = outputI['likeliness'] + 4
                                elif(attrOutSplit[1] == "angle"):
                                    outputI['likeliness'] = outputI['likeliness'] + 3
                                else:
                                    outputI['likeliness'] = outputI['likeliness'] + 1

        for i in range(0,6):
            if(len(attributesOfLikelyOutput) == (len(attributesOutputList[i]) - 1) and len(attributesOfLikelyOutput) == numAttrsMatched[i]):
                print "output is " + str(i+1)
                return str(i+1)
        output = 1
        similarity = 0

        for i in range(0,6):
            if((attributesOutputList[i]['likeliness'] > similarity)):
                similarity = attributesOutputList[i]['likeliness']
                output = i + 1
       
        print "Printing outputs"
        for i in range(0,6):
            print attributesOutputList[i]
        print "similarity = " + str(similarity)
        print "output is " + str(output)
        return str(output)

