# #!/usr/bin/python3

# def bresenham(start, end):

#     dx = end[0] - start[0]
#     dy = end[1] - start[1]

#     x_sign = 1 if dx > 0 else -1
#     y_sign = 1 if dy > 0 else -1

#     dx = abs(dx)
#     dy = abs(dy)

#     if dx > dy:
#         x_x = x_sign
#         x_y = 0
#         y_x = 0
#         y_y = y_sign
#     else:
#         dx = dy
#         dy = dx
#         x_x = 0
#         x_y = y_sign
#         y_x = x_sign
#         y_y = 0

#     D = 2*dy - dx
#     y = 0

#     for x in range(dx + 1):
#         yield start[0] + x*x_x + y*y_x, start[1] + x*x_y + y*y_y
#         if D >= 0:
#             y += 1
#             D -= 2*dx
#         D += 2*dy


import rospy


def bresenham_line(start, end, width):
    start = [int(start[0]/width), int(start[1]/width)]
    end = [int(end[0]/width), int(end[1]/width)]

    if abs(end[1]-start[1]) < abs(end[0]-start[0]):
        if start[0] > end[0]:
            return bresenhamLow(end, start, width)[::-1]
        else:
            return bresenhamLow(start, end, width)

    else:
        if start[1] > end[1]:
            return bresenhamHigh(end, start, width)[::-1]

        else:
            return bresenhamHigh(start, end, width)


def bresenhamLow(start, end, width):

    dx = end[0]-start[0]
    dy = end[1]-start[1]

    yi = 1
    if dy < 0:
        yi = -1
        dy = -dy

    D = 2*dy-dx

    x = start[0]
    y = start[1]

    points = []
    while x <= end[0]:
        points.append([x*width, y*width])
        # map[y,x]=1
        if D > 0:
            y += yi
            D += 2*(dy-dx)
        else:
            D += 2*dy

        x += 1

    return points


def bresenhamHigh(start, end, width):

    dx = end[0]-start[0]
    dy = end[1]-start[1]

    xi = 1
    if dx < 0:
        xi = -1
        dx = -dx

    D = 2*dx-dy

    x = start[0]
    y = start[1]

    points = []
    while y <= end[1]:
        points.append([x*width, y*width])
        if D > 0:
            x += xi
            D += 2*(dx-dy)
        else:
            D += 2*dx

        y += 1

    return points


# def bresenhamHigh(start, end, width):

#     dx= end[0]-start[0]
#     dy=end[1]-start[1]


#     xi=1
#     if dx<0:
#         xi=-1
#         xy=-xy

#     D=2*dx-dy

#     x=start[0]
#     y=start[1]

#     points=[]
#     while y<=end[1]:
#         points.append([x*width,y*width])

#         if D>0:
#             x+=xi
#             D+=2*(dx-dy)
#         else:
#             D+=2*dx

#         y+=1


#     return points

def bresenham_line(start, end, width):
    start = [int(start[0]/width), int(start[1]/width)]
    end = [int(end[0]/width), int(end[1]/width)]

    if abs(end[1]-start[1]) < abs(end[0]-start[0]):
        if start[0] > end[0]:
            return bresenhamLow(end, start, width)[::-1]
        else:
            return bresenhamLow(start, end, width)

    else:
        if start[1] > end[1]:
            return bresenhamHigh(end, start, width)[::-1]

        else:
            return bresenhamHigh(start, end, width)


def bresenhamLow(start, end, width):

    dx = end[0]-start[0]
    dy = end[1]-start[1]

    yi = 1
    if dy < 0:
        yi = -1
        dy = -dy

    D = 2*dy-dx

    x = start[0]
    y = start[1]

    points = []
    while x <= end[0]:
        points.append([x*width, y*width])
        # map[y,x]=1
        if D > 0:
            y += yi
            D += 2*(dy-dx)
        else:
            D += 2*dy

        x += 1

    return points


def bresenhamHigh(start, end, width):

    dx = end[0]-start[0]
    dy = end[1]-start[1]

    xi = 1
    if dx < 0:
        xi = -1
        dx = -dx

    D = 2*dx-dy

    x = start[0]
    y = start[1]

    points = []
    while y <= end[1]:
        points.append([x*width, y*width])
        if D > 0:
            x += xi
            D += 2*(dx-dy)
        else:
            D += 2*dx

        y += 1

    return points
