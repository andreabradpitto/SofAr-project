function ROgnew = strap_down(wg0,ROg,dt)
    skew = @(v)[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
    ROgnew=expm(skew(wg0) * dt)*ROg;
end