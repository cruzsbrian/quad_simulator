function main
    eta_0 = [0; 0; 0];
    omega_0 = [0; 0; 0];

    eta_d = [0; 0; 0];

    output = inner_loop(eta_d, eta_0, omega_0);

    disp(output);
end
