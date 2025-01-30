import numpy as np
import matplotlib.pyplot as plt
import follow_path as fp


# *0.02 scale by coordinate Fn
tolerance = 5
c_limit = 500

# Adaptive segmentation algorithm
def adaptive_parametric(x, y, tolerance):
    segments = []
    start_idx = 0

    while start_idx < len(x) - 1:
        for end_idx in range(start_idx + 1, len(x)):
            # Linear fit for x(t) and y(t)
            x1, x2 = x[start_idx], x[end_idx]
            y1, y2 = y[start_idx], y[end_idx]

            m_x = (x2 - x1) / (y2 - y1)
            c_x = x1 - m_x * y1

            m_y = (y2 - y1) / (x2 - x1)
            c_y = y1 - m_y * x1

            # Calculate error
            x_fit = m_x * y[start_idx:end_idx+1] + c_x
            y_fit = m_y * x[start_idx:end_idx+1] + c_y

            x_error = np.abs(x[start_idx:end_idx+1] - x_fit)
            y_error = np.abs(y[start_idx:end_idx+1] - y_fit)

            # Check tolerance
            if np.max(x_error) > tolerance and np.max(y_error) > tolerance:
                end_idx -= 1
                x1, x2 = x[start_idx], x[end_idx]
                y1, y2 = y[start_idx], y[end_idx]
                m_x = (x2 - x1) / (y2 - y1)
                c_x = x1 - m_x * y1
                m_y = (y2 - y1) / (x2 - x1)
                c_y = y1 - m_y * x1
                segments.append((x1, x2, m_y, c_y))
                start_idx = end_idx
                break
        else:
            x1, x2 = x[start_idx], x[-1]
            y1, y2 = y[start_idx], y[-1]
            m_x = (x2 - x1) / (y2 - y1)
            c_x = x1 - m_x * y1
            m_y = (y2 - y1) / (x2 - x1)
            c_y = y1 - m_y * x1
            segments.append((x1, x2, m_y, c_y))
            break

    return segments





pathCoord, iWallCoord, oWallCoord = fp.map_data()

plt.plot( iWallCoord[0, :c_limit], iWallCoord[1, :c_limit], label="inner_wall", color="blue")
plt.plot( oWallCoord[0, :c_limit], oWallCoord[1, :c_limit], label="outter_wall", color="blue")

iWall_segments = adaptive_parametric( iWallCoord[0, :c_limit], iWallCoord[1, :c_limit], tolerance *0.02)
oWall_segments = adaptive_parametric( oWallCoord[0, :c_limit], oWallCoord[1, :c_limit], tolerance *0.02)

for x1, x2, m, c in iWall_segments:
    plt.plot([x1, x2], [m*x1 + c, m*x2 + c], label=f"iWall_segments: {x1:.2f} to {x2:.2f}", color="red")

for x1, x2, m, c in oWall_segments:
    plt.plot([x1, x2], [m*x1 + c, m*x2 + c], label=f"oWall_segments: {x1:.2f} to {x2:.2f}", color="red")



plt.title("Optimized Linear Approximation")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
plt.show()


print("Linear Equations for Segments:")
for i, (x1, x2, m, c) in enumerate(iWall_segments):
    print(f"iWall_segments {i+1}: y = {m:.3f}x + {c:.3f}, for x in [{x1:.3f}, {x2:.3f}]")

for i, (x1, x2, m, c) in enumerate(oWall_segments):
    print(f"oWall_segments {i+1}: y = {m:.3f}x + {c:.3f}, for x in [{x1:.3f}, {x2:.3f}]")