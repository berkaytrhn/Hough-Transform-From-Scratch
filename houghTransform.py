import cv2 as cv
import numpy as np
import math
import sys
import os



def findSuccessRatio(estimatedLeft,estimatedRight,realLeft,realRight):

    #in case of selected reverse order, change estimatedLeft as top left corner and estimatedRight as bottom right corner
    tempEstimatedLeft = estimatedLeft
    tempEstimatedRight = estimatedRight
    estimatedLeft = (min(tempEstimatedLeft[0],tempEstimatedRight[0]),min(tempEstimatedLeft[1],tempEstimatedRight[1]))
    estimatedRight = (max(tempEstimatedLeft[0],tempEstimatedRight[0]),max(tempEstimatedLeft[1],tempEstimatedRight[1]))



    #area calculation
    firstArea = abs(estimatedLeft[0]-estimatedRight[0]) * abs(estimatedLeft[1]-estimatedRight[1])
    secondArea = abs(realLeft[0]-realRight[0]) * abs(realLeft[1]-realRight[1])

    #check if there is an intersect area
    if not (estimatedLeft[0] >= realRight[0] or estimatedRight[0]<=realLeft[0]):
        #can be intersection
        if not (estimatedLeft[1] >= realRight[1] or realLeft[1] >= estimatedRight[1]):
            #intersection proved
            side_1 = (min(estimatedRight[0],realRight[0]) - max(estimatedLeft[0],realLeft[0]))
            side_2 = (min(estimatedRight[1],realRight[1]) - max(estimatedLeft[1],realLeft[1]))
            overlapArea = side_1*side_2
            unionAre = firstArea+secondArea-overlapArea
            
            # can be uncommented to obtain area information
            #print("estimated : {} , real : {} , overlap : {}, union : {}".format(firstArea,secondArea,overlapArea,unionAre))

            iou = overlapArea/unionAre
            return iou
        else:
            return 0
    else:
        pass
        #no intersection      
        print("no overlap")
        return 0 

def readData(path):
    file = open(path)
    data = file.read()
    topLeft = (int(parseXML(data,"xmin")),int(parseXML(data,"ymin")))
    bottomRight = (int(parseXML(data,"xmax")),int(parseXML(data,"ymax")))
    return topLeft,bottomRight
    
def parseXML(data,tag):
    value = data.split("<{}>".format(tag))[1].split("</{}>".format(tag))[0]
    return value

def houghTransform(edgeDetectedImage,coloredImage,threshold,carName,showResult):
    
    

    accumulator,rhoValues,thetaValues,columnSize,rowSize = houghLines(edgeDetectedImage)
    
    
    #potential line filtering from accumulator
    horizontalPoints,verticalPoints = potentialLineSelection(accumulator,rhoValues,thetaValues,threshold,coloredImage)

    

    # find intersection points in horizontal and vertical lines, intersection points have to be inside image very high probability
    # store that intersection points in array to obtain possible rectangle corner coordinates.
    intersectionCoordinates = []
    
    #find intersection coordinates
    findIntersectionPoints(horizontalPoints,verticalPoints,intersectionCoordinates,coloredImage)


            


    #determining points
    firstPointCandidates = []
    secondPointCandidates = []

    #old boundaries first -> 45,25,50,30, new -> 50,25,60,40
    #old boundaries second -> 80,55,70,50 new -> 75,50,90,60
    selectCandidate(intersectionCoordinates,firstPointCandidates,secondPointCandidates,columnSize,rowSize,[50,25,60,40,75,50,90,60])




    # we can remove selected points to avoid selecting same point twice but, 
    # in the below lines we will see that algorithm controls points according to car plate ratio 
    # which avoids selecting same points



    #algorithm based on pixel percent approximation
    firstPoint,secondPoint = candidateFilterBasedOnMinimumDifference(firstPointCandidates,secondPointCandidates,intersectionCoordinates,2.3)
    
    #algorithm based on real average 
    #firstPoint,secondPoint = candidateFilterBasedOnRealAverage(firstPointCandidates,secondPointCandidates,intersectionCoordinates,2.3)

    
    
    print("First Selected Point : ",firstPoint)
    print("Second Selected Point : ",secondPoint)

    #drawing rectangle
    cv.rectangle(coloredImage,firstPoint,secondPoint,(255,0,0),2)   

    #can be uncommented to see edge detected version and rectangle drawed version separately
    #cv.imwrite("./detected.png".format(carName),coloredImage)
    #cv.imwrite("./edge.png",edgeDetectedImage)

    #if this function called from generalSuccessTest, showResult value will be false
    # otherwise it will be true for showing result to user on individual process 
    if showResult:
        cv.resizeWindow("Result Image",700,700)
        cv.imshow("Result Image",coloredImage)
        cv.waitKey(0)
    
    
    realPoint_1,realPoint_2 = readData("./annotations/{}.xml".format(carName))
    iou = findSuccessRatio(firstPoint,secondPoint,realPoint_1,realPoint_2)
    print("individual percent : %",iou*100)
    print("Real Points : ",realPoint_1,",",realPoint_2)
    
    #can be uncommented for general success test  
    #cv.imwrite("./{}/{}_detected_{}.png".format("directory_1",carName,round(iou*100,2)),coloredImage)

    #iou*100,
    return coloredImage

def houghLines(edgeDetectedImage):
    rowSize = len(edgeDetectedImage)
    columnSize = len(edgeDetectedImage[0])
    diagonalLength = round(np.sqrt(rowSize**2 + columnSize**2))+1
    #calculating arrays which will be used     
    rhoValues = np.arange(-diagonalLength,diagonalLength)
    thetaValues = np.deg2rad(np.arange(-90,90))
    accumulator = np.zeros((len(rhoValues),len(thetaValues)))
    print(rowSize,columnSize,sep="x")
    #creating sin and cos numpy vectors to access theta values easily later
    sinVector = np.sin(thetaValues)
    cosVector = np.cos(thetaValues)
    #getting vector of both column and row indexes
    rowIndexes,columnIndexes = np.nonzero(edgeDetectedImage)
    #voting for line
    for coordinateIndex in range(len(rowIndexes)):
        yCoordinate = rowIndexes[coordinateIndex]
        xCoordinate = columnIndexes[coordinateIndex]
        for thetaIndex in range(len(thetaValues)):
            rhoValue = round(xCoordinate*cosVector[thetaIndex]+yCoordinate*sinVector[thetaIndex])
            #operation for indexing rho
            rhoIndex = rhoValue+diagonalLength
            accumulator[rhoIndex,thetaIndex] += 1
    return accumulator,rhoValues,thetaValues,columnSize,rowSize

def potentialLineSelection(accumulator,rhoValues,thetaValues,threshold,coloredImage):
    # selecting lines with some offset, accepts lines with theta as 78<theta<102,-102<theta<-78 or -10<theta<10
    # after every 15 pixel both vectical and horizontal
    horizontalPoints = []
    verticalPoints = []
    
    previousHorizontal=None
    previousVertical=None
    for row in range(len(accumulator)):
        for column in range(len(accumulator[row])):
            rho  = rhoValues[row]
            theta = thetaValues[column]
            
            #selecting nearly 90 or -90 or 0 degree thetas 
            if abs(np.rad2deg(theta))<10 or abs(np.rad2deg(theta)-90)<12 or abs(np.rad2deg(theta)+90)<12:
            

                #change threshold according to degree because horizontal axis of plate is bigger than vertical axis
                if abs(np.rad2deg(theta))<10:
                    controlThreshold = threshold*2/5
                else:
                    controlThreshold=threshold

                if accumulator[row][column]>controlThreshold:

                    #normalizations, converting hough space coordinates to image space coordinates but little bit outer than image
                    x0 = math.cos(theta) * rho
                    y0 = math.sin(theta) * rho
                    sinTheta = math.sin(theta)
                    cosTheta = math.cos(theta)

                    #two end points of corresponding line which are outside of image boundaries
                    pt1 = (int(x0 + 1000*(-sinTheta)), int(y0 + 1000*(cosTheta)))
                    pt2 = (int(x0 - 1000*(-sinTheta)), int(y0 - 1000*(cosTheta)))
                    
                    # controlling previous line to avoid collapse according to offset value which is 15
                    # after, add them in corresponding horizontal or vertical array to use later
                    if controlThreshold==threshold:
                        if previousHorizontal!=None:
                            ppt1 = previousHorizontal[0]
                            ppt2 = previousHorizontal[1]
                            
                            currentMiddlePoint = [(pt2[0]+pt1[0])/2,(pt2[1]+pt1[1])/2]
                            previousMiddlePoint = [(ppt2[0]+ppt1[0])/2,(ppt2[1]+ppt1[1])/2]
                            if abs(currentMiddlePoint[1]-previousMiddlePoint[1])<15:
                                continue

                    elif controlThreshold == threshold*2/5:
                        if previousVertical!=None:
                            ppt1 = previousVertical[0]
                            ppt2 = previousVertical[1] 

                            currentMiddlePoint = [(pt2[0]+pt1[0])/2,(pt2[1]+pt1[1])/2]
                            previousMiddlePoint = [(ppt2[0]+ppt1[0])/2,(ppt2[1]+ppt1[1])/2]
                            if abs(currentMiddlePoint[0]-previousMiddlePoint[0])<15:
                                continue

                    if controlThreshold == threshold*2/5:
                        verticalPoints.append([pt1,pt2])
                        previousVertical=[pt1,pt2]
                    elif controlThreshold==threshold:
                        horizontalPoints.append([pt1,pt2])
                        previousHorizontal=[pt1,pt2]
                    #drawing filtered lines, can be uncommented
                    #cv.line(coloredImage, pt1, pt2, (0,0,255), 1)
    return horizontalPoints,verticalPoints

def findIntersectionPoints(horizontalPoints,verticalPoints,intersectionCoordinates,coloredImage):
    for pointPair in horizontalPoints:
        pt1 = pointPair[0]
        pt2 = pointPair[1]
        
        #first line slope and intercept
        slope_1 = (pt2[1]-pt1[1])/(pt2[0]-pt1[0]+0.1)
        intercept_1 = pt2[1]- slope_1*pt2[0] 

        for secondPointPair in verticalPoints:
            spt1 = secondPointPair[0]
            spt2 = secondPointPair[1]

            #second line slope and intercept
            slope_2 = (spt2[1]-spt1[1])/(spt2[0]-spt1[0]+0.1)
            intercept_2 = spt2[1]- slope_2*spt2[0]

            #intersection point calculation
            xValue = - (intercept_1-intercept_2)/(slope_1-slope_2)
            yValue = xValue*slope_1 + intercept_1
            
            xValue = round(xValue)
            yValue = round(yValue)

            #store coordinates [0]=columnIndex,[1]=rowIndex
            intersectionCoordinates.append((xValue,yValue))

            
            try:
                # painting intersection points to green
                # can be uncommented if want, does not effect result
                #coloredImage[yValue][xValue] = [0,255,0]
                pass
            except IndexError:
                #if it is out of bounds it is useless and we can ignore it
                pass

def selectCandidate(intersectionCoordinates,firstPointCandidates,secondPointCandidates,columnSize,rowSize,boundaries):
    #first point
    for point in intersectionCoordinates:
        x = point[0]
        y = point[1]

        xRatio = (x/columnSize*100)
        yRatio = (y/rowSize*100)
        if (xRatio<boundaries[0] and xRatio>boundaries[1]) and (yRatio<boundaries[2] and yRatio>boundaries[3]):
            #g. boundaries 50,25,60,40
            firstPointCandidates.append(point)
    #second point
    for point in intersectionCoordinates:
        x = point[0]
        y = point[1]

        xRatio = (x/columnSize*100)
        yRatio = (y/rowSize*100)
        if (xRatio<boundaries[4] and xRatio>boundaries[5]) and (yRatio<boundaries[6] and yRatio>boundaries[7]):
            #g. boundaries -> 75,50,90,60
            secondPointCandidates.append(point)

def candidateFilterBasedOnRealAverage(firstPointCandidates,secondPointCandidates,intersectionCoordinates,ratio):
    firstPoint = None
    secondPoint = None
    
    if len(firstPointCandidates) == 0 or len(secondPointCandidates) == 0:
        #incase of not enough lines to find and select intersect points to draw rectangle
        
        #can be uncommented to check if this line block is executed or not
        #print("One of candidate array is empty!, applying alternative algorithm")
        minScore = 42000
        for point_1 in intersectionCoordinates:
            x_1,y_1 = point_1[0],point_1[1]
            for point_2 in intersectionCoordinates:
                x_2,y_2 = point_2[0],point_2[1]
                score = abs((x_2-x_1)/(y_2-y_1+0.1)-ratio)
                #print(score)
                if minScore>score:
                    minScore=score
                    firstPoint=point_1
                    secondPoint=point_2
    else:
        #new algortihm based on average of coordinates
        minScore = 42000
        for point_1 in firstPointCandidates:
            x_1,y_1 = point_1[0],point_1[1]
            for point_2 in secondPointCandidates:
                x_2,y_2 = point_2[0],point_2[1]
                xWiseScore_1 = abs(x_1 - 174) #average x_1
                yWiseScore_1 = abs(y_1 - 160) #average y_1
                xWiseScore_2 = abs(x_2 - 280) #average x_2
                yWiseScore_2 = abs(y_2 - 199) #average y_2
                score = xWiseScore_1+yWiseScore_1+xWiseScore_2+yWiseScore_2
                if minScore>score:
                    minScore=score
                    firstPoint=point_1
                    secondPoint=point_2
    return firstPoint,secondPoint

def candidateFilterBasedOnMinimumDifference(firstPointCandidates,secondPointCandidates,intersectionCoordinates,ratio):
    firstPoint = None
    secondPoint = None
    
    
    if len(firstPointCandidates) == 0 or len(secondPointCandidates) == 0:
        #incase of not enough lines to find and select intersect points to draw rectangle
        minScore = 42000
        print("one of candidate array is empty")
        for point_1 in intersectionCoordinates:
            x_1,y_1 = point_1[0],point_1[1]
            for point_2 in intersectionCoordinates:
                x_2,y_2 = point_2[0],point_2[1]
                score = abs((x_2-x_1)/(y_2-y_1+0.1)-ratio)
                #print(score)
                if minScore>score:
                    minScore=score
                    firstPoint=point_1
                    secondPoint=point_2
    else:
        #old algorithm based on ratio score minimize
        minScore = 42000
        for point_1 in firstPointCandidates:
            x_1,y_1 = point_1[0],point_1[1]
            for point_2 in secondPointCandidates:
                x_2,y_2 = point_2[0],point_2[1]
                score = abs((x_2-x_1)/(y_2-y_1)-ratio)
                #print(score)
                if minScore>score:
                    minScore=score
                    firstPoint=point_1
                    secondPoint=point_2
        
    return firstPoint,secondPoint

def coordinateAverageCalculator(interval):
    start = int(interval.split("-")[0])
    end = int(interval.split("-")[1])
    topLeftXTotal=0
    topLeftYTotal=0
    rightBottomXTotal=0
    rightBottomYTotal=0
    pointCount=0
    for number in range(start,end+1):
        path = "./annotations/Cars{}.xml".format(number)
        file = open(path)
        data = file.read()

        topLeftXTotal+=int(parseXML(data,"xmin"))
        topLeftYTotal+=int(parseXML(data,"ymin"))
        rightBottomXTotal+=int(parseXML(data,"xmax"))
        rightBottomYTotal+=int(parseXML(data,"ymax"))
        pointCount+=1
    

    topleftXratio = round(topLeftXTotal/pointCount)
    topleftYratio = round(topLeftYTotal/pointCount)
    rightbottomXratio = round(rightBottomXTotal/pointCount)
    rightbottomYratio = round(rightBottomYTotal/pointCount)

    print("Topleft Average : ({},{})\nRightBottomAverage : ({},{})".format(topleftXratio,topleftYratio,rightbottomXratio,rightbottomYratio))

def generalSuccessTest(interval):
    start = int(interval.split("-")[0])
    end = int(interval.split("-")[1])
    dataCount=0
    totalIOU=0
    zeroCount=0
    for index in range(start,end+1):
        print("******\n",index," numbered data...")
        image = cv.imread("./images/Cars{}.png".format(index))
        grayScale = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        edges = cv.Canny(grayScale,70,175,apertureSize=3) 

        
        iouValue  = houghTransform(edges,image,70,"Cars{}".format(index),False)
        totalIOU+=iouValue
        dataCount+=1
        #increment zero counter
        if iouValue==0:
            zeroCount+=1
        print("Current percent : %{}".format(totalIOU/dataCount))

    print("Number of zero ratio estimations : {}".format(zeroCount))
    print("General Success average percent : %{}".format(totalIOU/dataCount))

def main():
    def processIndividualImage(imageNumber):
        

        #reading image and obtain grayscale one also
        image = cv.imread("./images/Cars{}.png".format(imageNumber))
        grayScale = cv.cvtColor(image,cv.COLOR_BGR2GRAY)

        #applying canny edge detection
        edges = cv.Canny(grayScale,70,175,apertureSize=3)

        #can be uncommented to see edge detected image
        #cv.imshow("edges",edges)

        #calling general function for one specified image
        houghTransform(edges,image,70,"Cars{}".format(imageNumber),True)
    
    print("-----------Berkay's Hough Transform-----------")
    while True:
        
        choice  =int(input("\n1) General Success Test\n2) Individual Success Test and Result\nSelect one : "))
        if choice==1:
            interval = input("Type an interval(start-end): ")
            generalSuccessTest(interval)
            break
        elif choice == 2:
            number = input("Type a number for specific image : ")
            processIndividualImage(number)
            break
        else:
            print("Invalid Choice!!")
            continue   
    
main()