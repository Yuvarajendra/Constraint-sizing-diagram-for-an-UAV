clear all;
close all;
clc;

neta=0.85                           %% Overall efficiency of the systems in an aircraft
g=9.81;
Sg=30.48;                           %% Ground roll of about 100 ft
AR=8;                               %% Aspect ratio
b=1.85;                             %% Wing span
S=(b^2)/AR;                         %% Wetted area
MTOW=15*9.81;                       %% Total weight
Density=0.8881;                     %% Density at 4000 ft
Cl_cruise=0.9;                      %% Cruise coefficient of lift
Cl_max=1.3;
Cd_cruise=0.027;                    %% Cruise coefficient of drag
Cd_TO=0.040;                        %% Take-off coefficient of drag
Cl_TO=0.7;                          %% Take-off coefficient of lift
V_cruise=sqrt((2*MTOW)/(Density*S*Cl_cruise));
V_service_ceiling=0.508;            %% Vertical speed in 100 ft/min
e=1.78*(1-0.045*(8)^0.68)-0.64;     %% Oswalds efficiency factor
k=1/(pi*e*AR);                      %% Lift induced drag factor
q=(0.5*Density*V_cruise^2);         %% Dynamic pressure
n=2;                                %% Load factor
V_vertical_speed=3.048;             %% Vertical speed in 25 ft/sec
V_stall=sqrt((2/Density)*(MTOW/S)*(1/Cl_max))               %% Stall speed
V_LOF=1.10*V_stall;                 %% Lift-off speed, 110% of stall speed
q_takeoff=(0.5*Density*(V_LOF/sqrt(2))^2);
mu=0.5;                             %% Ground friction coefficient

W_S=linspace(0,400,20);

for(i=1:1:length(W_S))
    
T_W_cruise_speed(i)=(q*Cd_cruise*(1/W_S(i))+k*(1/q)*(W_S(i)))*(V_cruise/(neta*550));

T_W_Climb(i)=((V_vertical_speed/V_cruise)+(q/W_S(i))*Cd_cruise+(k/q)*(W_S(i)))*(V_cruise/(neta*550));

T_W_constant_veloc_turn(i)=(q*((Cd_cruise/W_S(i))+k*(n/q)^2 *(W_S(i))))*(V_cruise/(neta*550));

T_W_service(i)= ((V_service_ceiling/sqrt((2/Density)*W_S(i)*sqrt(k/(3*Cd_cruise))))+4*sqrt((k*Cd_cruise)/3))*(V_cruise/(neta*550));

T_W_takeoff(i)=((V_LOF^2/(2*g*Sg))+((q_takeoff*Cd_TO)/W_S(i)) + mu*(1-((q_takeoff*Cl_TO)/W_S(i))))*(V_cruise/(neta*550));
end
plot(W_S,T_W_cruise_speed,'Linewidth',2)
hold on
plot(W_S,T_W_Climb,'Linewidth',2)
hold on
plot(W_S,T_W_constant_veloc_turn,'Linewidth',2)
hold on
plot(W_S,T_W_service,'Linewidth',2)
% hold on
% plot(W_S,T_W_takeoff,'Linewidth',2)

xlabel('Wing loading [N/m^2]')
ylabel('Power to Weight ratio')
legend('Cruise speed','Rate of climb','Constant velocity turn','Service ceiling','Take-off')


%%% Stall speeds
Stall_speeds=10:5:25;
for(k=1:1:length(W_S))
    
for(j=1:1:length(Stall_speeds))    
Cl_maxx(k,j)=(W_S(k))*(1/(0.5*Density*Stall_speeds(j)^2));
end

end
yyaxis right;
plot(W_S,Cl_maxx,'k','Linewidth',1.5)
ylabel('Required Clmax')
legend('Cruise speed','Rate of climb','Constant velocity turn','Service ceiling','Vs=10 m/s','Vs=15 m/s','Vs=20 m/s','Vs=25 m/s')