function alpha  = calculate_alpha(t)

    alpha_initial = 0.0000005;
    alpha_final = 0.5;

    alpha = alpha_initial * (500*t)^2;

    alpha = max(min(alpha, alpha_final), alpha_initial);


end