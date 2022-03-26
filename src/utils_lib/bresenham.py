#!/usr/bin/python3

# def bresenham(x0, y0, x1, y1):
#     """Yield integer coordinates on the line from (x0, y0) to (x1, y1).
#     Input coordinates should be integers.
#     The result will contain both the start and the end point.
#     """
#     dx = x1 - x0
#     dy = y1 - y0

#     xsign = 1 if dx > 0 else -1
#     ysign = 1 if dy > 0 else -1

#     dx = abs(dx)
#     dy = abs(dy)

#     if dx > dy:
#         xx, xy, yx, yy = xsign, 0, 0, ysign
#     else:
#         dx, dy = dy, dx
#         xx, xy, yx, yy = 0, ysign, xsign, 0

#     D = 2*dy - dx
#     y = 0

#     for x in range(dx + 1):
#         yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
#         if D >= 0:
#             y += 1
#             D -= 2*dx
#         D += 2*dy


#!/usr/bin/python3

def line_high(start, end, width, points):
    dx = end[0]-start[0]
    dy = end[1]-start[1]
    xi = width
    if dx < 0:
        xi = -width
        dx = -dx
    D = (2*dx) - dy
    x = start[0]

    for y in range(start[1], end[1]+width):
        points.append([x, y])
        if D > 0:
            x = x + xi
            D = D + (2*(dx-dy))
        else:
            D = D + (2*dx)
    return points


def line_low(start, end, width, points):
    dx = end[0]-start[0]
    dy = end[1]-start[1]
    yi = width
    if dy < 0:
        yi = -width
        dy = -dy
    D = (2*dy) - dy
    y = start[1]

    for x in range(start[0], end[0]+width):
        points.append([x, y])
        if D > 0:
            y = y + yi
            D = D + (2*(dy-dx))
        else:
            D = D + (2*dy)
    return points


def bresenham_line(start, end, width=1):
    # start:  2D lists
    # end:    2D lists
    # width:  scalar with resolution of grid map
    # points: list of 2D points from start to end
    points = []
    if abs(end[1]-start[1]) < abs(end[0]-start[0]):

        if start[0] > end[0]:
            line_low(end, start, width, points)
        else:
            line_low(start, end, width, points)
    else:
        if start[1] > end[1]:
            line_high(end, width, start)
        else:
            line_high(start, width, end)

    return points
