% Lab 3: Closed Loop Response of Control Systems

%% Tracking
s = tf('s');
G = 59.292/(s^2 +6.978*s+15.123);   % Controller tf
den = ([1, 6.978, 15.123]);
%(1)
figure;
K = 3;
r = roots(den)
bode(K*G)
figure;
%(2) 
Y = minreal((G*K)/(1+G*K));           % Factor the function
s0 = 0;                               % plug in to error function
E = minreal((1/(1+G*K))*(1/s)*(s));   % error tf 
evalfr(E,s0)                          % final Values theorem
t = linspace(0,10,1000);

[y_track,t_track]=step(Y); 
plot(y_track)
title('step repsonse')
% (3)
K = 10;

Y = minreal((G*K)/(1+G*K));

E = minreal(1/(1+G*K)*(1/s)); % Error function (s)*(1/s) = 1
s0=0;

evalfr(minreal(s*E), s0) %Evaluate the steady-state tracking error when K=3
figure;
% Plotting part for the step response
step(Y) % Plot the step response
figure;

bode(K*G) % Plot the bode plot
figure
bode(K*G)


K = 3;
G = 59.292/(s^2+6.978*s+15.123);
R = 1/s;
F = 1/G;
Y = ((F*G+K*G)/(1+K*G));


E =(1-Y)*R;
evalfr(minreal(s*E),0)
step(Y)
figure

%(4)
s = tf('s');
K = 3+2/s;                            % assuming its a step 
G = 59.292/(s^2 +6.978*s+15.123);     % Controller tf

Y = minreal((G*K)/(1+G*K));           % Factor the function
s0 = 0;                               % plug in to error function
E = minreal((1/(1+G*K))*(1/s)*(s));   % error tf 
evalfr(E,s0)                          % final Values theorem
step(Y)

s = tf('s');
K = 3+10/s;                           % assuming its a step 
G = 59.292/(s^2 +6.978*s+15.123);     % Controller tf
num = 59.292;
den = ([1, 6.978, 15.123]);
 
Y = minreal((G*K)/(1+G*K));           % Factor the function
s0 = 0;                               % plug in to error function
E = minreal((1/(1+G*K))*(1/s)*(s));   % error tf 
evalfr(E,s0)                          % final Values theorem
t = linspace(0,10,1000);
step(Y)
%% Disturbance Rejection

s = tf('s');
R=0;
F=0;
W=1;
s0=0;
G = 59.292/(s^2 +6.978*s+15.123);     % G transfer function
t = linspace(0,2,100);

%(1)
K = 3;
figure;
Y = (1/(1+G*K))*(W+G*K*R+F*W*G);
E = (R-Y);
ess_21 = evalfr(minreal(s*E),s0)
step(Y,t)
title('2 part 1');

%(2)
K = 10;
Y = (1/(1+G*K))*(W+G*K*R+F*W*G);
E = minreal(R-Y);
ess_22 = evalfr(minreal(s*E),s0)     % final Values theorem
figure;
step(Y,t)
title('2 part 2');

%(3)
K =3;                                % P control
G = 59.292/(s^2 +6.978*s+15.123);    % G transfer function
G0 = 59.292./15.123;                 % G(0)
F = (-1/G0)-(K*R)/W; 
Y = (1/(1+G*K))*(W + G*K*R+F*W*G);
E = minreal(R-Y);

ess_23 = evalfr(s*E,s0)              % final Values theorem
figure;
step(Y)
title('2 part 3');

% (4)
K =3 + 2/s;                          % P control
G = 59.292/(s^2 +6.978*s+15.123);    % G transfer function
G0 = 59.292./15.123;                 % G(0)
W = 1; 
Y = (1/(1+G*K))*(W + G*K*R+F*W*G);
E = minreal(R-Y);

ess_24 = evalfr(s*E,s0)              % final Values theorem
figure;
step(Y)
title('2 part 4');

% (5)
K =3 + 10/s;                         % P control
G = 59.292/(s^2 +6.978*s+15.123);    % G transfer function
G0 = 59.292./15.123;                 % G(0)
W = 1; 
Y = (1/(1+G*K))*(W + G*K*R+F*W*G);
E = minreal(R-Y);

ess_25 = evalfr(s*E,s0)              % final Values theorem
figure;
step(Y)
title('2 part 5');
%% Stability through Dynamic Pole-Zero Maps
% 3.1 system 1
%(1) - open loop transfer function
s = tf('s');
G = 1/(s*(s+2)^2);
den = [1 4 4 0];
r = roots(den);
plot(real(r),imag(r),'kx','MarkerSize',10,'LineWidth',2)
title('pole zero map');
grid on
hold on

%(2) - Determine values for K
% K>0 && K<16

%(3) - closed loop
k = [1,4,16,50];
for i=1:length(k)
    den = [1 4 4 k(i)];
    r = roots(den);
    for j=1:length(r)
        ri =(r(j));
        if real(ri)>0       % unstable
            plot(real(ri),imag(ri),'rx','MarkerSize',10,'LineWidth',2)
        end
        if real(ri)<0       % stable
            plot(real(ri),imag(ri),'gx','MarkerSize',10,'LineWidth',2)
        end
        if real(ri) <= 0.1 && real(ri)>= -.1 % marginally stable
            plot(real(ri),imag(ri),'bx','MarkerSize',10,'LineWidth',2)
        end   
    end
end

% (4)
% call pole function
%k = logspace(0,3,1000);
K = tf(k,1);
G = tf(1,[1 4 4 0]);
%pole_func(G,K);
hold off;

figure;

% 3.2 System 2
%(1)
num = [1 1];
G = 1/(s-1)^2;
den = [1 -2 1];
zero = roots(num);
r = roots(den);
plot(real(r),imag(r),'kx','MarkerSize',10,'LineWidth',2);
grid on;
hold on;
plot(real(zero),imag(zero),'ko','MarkerSize',10,'LineWidth',2)
title('pole zero map');


% (2)
%k>-1
%k<-2

% (3)
k = [0.1 1 2 4 10];
for i=1:length(k)
    T = (k(i)*(s+1)*G)/(1+k(i)*(s+1)*G);
    [num,den] = tfdata(T,'v');
    r = roots(den);
    for j=1:length(r)
        ri =(r(j));
        if real(ri)>0       % unstable
            plot(real(ri),imag(ri),'rx','MarkerSize',10,'LineWidth',2)
        end
        if real(ri)<0       % stable
            plot(real(ri),imag(ri),'gx','MarkerSize',10,'LineWidth',2)
        end
        if real(ri)<= 0.1 && real(ri)>= -.1 % marginally stable
            plot(real(ri),imag(ri),'bx','MarkerSize',10,'LineWidth',2)
        end   
    end
end

% (4)
%k = logspace(0,3,1000);
K = tf([k 1],1);
G = tf(1,[1 -2 1]);
%pole_func(G,K);
hold off;
%% 4. Bode Plots and Stability 
figure;
k_3_1 = [1 4 16 50];
den = [1 4 4 0];
for a=1:length(k_3_1)
    b = k_3_1(a);
    G = (b) * tf(1,den);
    bode(G);
    hold on;
end
legend('k=1', 'k=4', 'k=16','k=50');

hold off;

figure;
k_3_2 = [.1 1 2 4 10];

for c=1:length(k_3_2)
    d=k_3_2(c)*(s+1);
    G = (d) * tf(1,[1 -2 1]);
    bode(G);
    hold on;
end
legend('k=.1', 'k=1', 'k=2', 'k=4', 'k=10');
%% Funcitons
% function  pole_func(G,k)
% rlocusplot(G,k)
% end
