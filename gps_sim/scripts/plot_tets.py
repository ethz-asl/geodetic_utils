
if __name__ == "__main__":
    # RTK:
    # a = GpsNoiser(white=np.array([0.0005, 0.0005, 0.002]),
    #              pink=np.array([0.0035, 0.0035, 0.007]),
    #              brown=np.array([0.001, 0.001, 0.002]),
    #              epsilons=np.array([0, 1, 2]))

    # SPP:
    a = GpsNoiser(white=np.array([0.0075, 0.0075, 0.015]),
                  pink=np.array([0.001, 0.001, 0.002]),
                  brown=np.array([0.03, 0.03, 0.06]),
                  epsilons=np.array([0, 1, 3.0]))

    a.sample_noise()

    # load true rtk data.
    name = "FLOAT"
    rtk_data = np.load("/home/mpantic/Work/ARMA/GPS/bags/bag_sim_gps/spp_2021-02-18-16-31-50.bag_np.npy")
    # rtk_data = np.load("/home/mpantic/Work/ARMA/GPS/bags/bag_sim_gps/rtk_2021-02-18-16-21-18.bag_np.npy")
    rtk_data -= rtk_data.mean(axis=0)

    # generate fake data of same length
    sim_data = np.zeros(rtk_data.shape)
    for i in range(0, len(rtk_data)):
        sim_data[i, :] = a.perturb(np.array([0.0, 0.0, 0.0]))

    print("empirical covariance")
    print(np.cov(sim_data.T))

    exit(0)

    wa = 2  # working axis
    s_true, f_true = mlab.psd(rtk_data[:, wa], NFFT=2 ** 12)
    s_sim, f_sim = mlab.psd(sim_data[:, wa], NFFT=2 ** 12)

    plt.figure(figsize=[6, 6])
    plt.loglog(f_true, s_true, 'r', alpha=0.25)
    plt.loglog(f_sim, s_sim, 'b', alpha=0.25)
    plt.xlim([10 ** -3.5, 1])
    plt.ylim([10 ** -9, 10 ** 3])
    plt.title("Noise Power Spectrum " + name + " (blue=sim/red=real data)")
    plt.xlabel("Freq")
    plt.ylabel("Amp")
    plt.grid(True)
    plt.savefig("power_spec_" + name + ".png")

    plt.figure(figsize=[6, 6])
    plt.scatter(rtk_data[:, 0], rtk_data[:, 2], c=np.arange(len(rtk_data)), marker="x")
    plt.title("Bias Free " + name + " ENU Position (REAL DATA)")
    plt.xlabel("x_enu [m]")
    plt.ylabel("y_enu [m]")
    plt.grid(True)
    plt.savefig("enu_xy_" + name + "_real.png")

    plt.figure(figsize=[6, 6])
    plt.scatter(sim_data[:, 0], sim_data[:, 2], c=np.arange(len(sim_data)), marker="x")
    plt.title("Bias Free " + name + " ENU Position (SIM DATA)")
    plt.xlabel("x_enu [m]")
    plt.ylabel("y_enu [m]")
    plt.grid(True)
    plt.savefig("enu_xy_" + name + "_sim.png")