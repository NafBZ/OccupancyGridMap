#!/usr/bin/python3


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
