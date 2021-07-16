import numpy as np
import matplotlib.pyplot as plt
import os
import imageio
import utils

g = 9.81


def vectorMod(x, y):
    return np.sqrt(x**2 + y**2)


def drawFrame(j, v0, x, z, u, w, degrees, filenames, saveRoute):
    speed = round(vectorMod(u[j], w[j]), 3)

    fig, ax = plt.subplots()

    ax.plot(x[j], z[j], 'bo', linewidth=5, markersize=5)
    ax.plot(x[0:j], z[0:j], 'go', linewidth=1, markersize=1)

    ax.plot(range(-1000, 1000), [z[j] for i in range(-1000, 1000)], linewidth=1)
    ax.plot([x[j] for i in range(-1000, 1000)], range(-1000, 1000), linewidth=1)

    ax.text(0.7, 0.9, "x: " + str(round(x[j], 3)) + "\ny: " + str(round(z[j], 3)), fontsize="small",
            transform=ax.transAxes)
    ax.text(0.7, 0.85, "Current velocity: " + str(speed), fontsize="small",
            transform=ax.transAxes)

    ax.text(0.05, 0.9, "Initial Angle: " + str(degrees), fontsize="small",
            transform=ax.transAxes)
    ax.text(0.05, 0.85, "Initial Velocity: " + str(v0), fontsize="small",
            transform=ax.transAxes)

    plt.xlabel('x')
    plt.ylabel('z')
    plt.xlim(0, 100)
    plt.ylim(0, 100)

    filename = saveRoute + '/image' + str(j) + ".jpg"
    filenames.append(filename)

    plt.savefig(filename, dpi=200)  # quality of end result
    plt.close()


def findTrajectory(v0, degrees, d, z0=0, x0=0, res=0, M=1000, saveRoute="temp"):
    nameTemplate = str(v0) + "v-" + str(degrees) + 'deg' + str(z0) + "height"

    if degrees == 0: degrees = 10e-10
    angle = degrees * np.pi / 180

    # file name list (used when creating a gif)
    filenames = []

    # velocities
    u = np.zeros(M)
    w = np.zeros(M)

    x = np.zeros(M)
    z = np.zeros(M)
    x[0] = x0
    z[0] = z0

    # velocities at the start
    u[0] = v0 * np.cos(angle)
    w[0] = v0 * np.sin(angle)

    # all the points in the trajectory go here
    pointArr = []

    # flight time
    T = (w[0]/g) * (1 + np.sqrt(1 + (2 * z0 * g) / w[0] ** 2))
    tau = T / M

    try:
        os.mkdir(saveRoute)
    except:
        pass

    # Some boring formulas
    for j in range(M-1):
        velocity_vector_mod = np.sqrt(u[j] ** 2 + w[j] ** 2)
        u[j + 1] = u[j] - tau * res * 1.0 * (3 / 4) * (u[j] * velocity_vector_mod) / d
        w[j + 1] = w[j] - tau * (g + res * 1.0 * (3 / 4) * (w[j] * velocity_vector_mod) / d)
        x[j + 1] = x[j] + tau * u[j]
        z[j + 1] = z[j] + tau * w[j]

    for j in range(M):
        if z[j] < 0:
            break

        drawFrame(j, v0, x, z, u, w, degrees, filenames, saveRoute)
        utils.printProgressBar(j, M-1, prefix="Generating images...", length=60)
        if j == M-1:
            for i in range(60):
                drawFrame(j, v0, x, z, u, w, degrees, filenames, saveRoute)

    images = []
    for filename in filenames:
        images.append(imageio.imread(filename))

    print("Generating GIF...")
    params = {'duration': 1/(M/3)}
    imageio.mimsave(nameTemplate + '.gif', images, **params);

    print('Done :)')


def main():
    print("Welcome to projectile motion visualizer! Please input initial conditions");
    v0 = float(input("Initial speed: "))
    angle = float(input("Angle: "))
    diameter = float(input("Diameter: "))
    height = float(input("Height: "))
    pointCount = int(input("Point count (recommended below 100): "))

    findTrajectory(v0, angle, diameter, height, M=pointCount)


if __name__ == "__main__":
    main()