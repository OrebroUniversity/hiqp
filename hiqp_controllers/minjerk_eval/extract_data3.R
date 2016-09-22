library(ggplot2)

tf <- 10
e0 <- 0.0667979
C <- 40/sqrt(3) * e0/(tf*tf)

rawdata <- read.csv("csv_data/f20t10K0.csv", header=FALSE)
J <- rawdata[rawdata$V2=='J',3:20]
qdot <- rawdata[rawdata$V2=='qdot',3:20]
t <- rawdata[rawdata$V2=='J',c("V1")]

dt <- t[2:length(t)] - t[1:length(t)-1]
dt <- c(dt, sum(dt)/length(dt))

t_red <- t[2:(length(t)-1)]
dt_red <- dt[2:(length(dt)-1)]

de <- rowSums(J*qdot)
de <- unname(de)
dde <- (-0.5*de[3:length(de)] +0.5*de[1:(length(de)-2)]) / dt_red
ddde <- (de[3:length(de)] -2*de[2:(length(de)-1)] +de[1:(length(de)-2)]) / dt_red

ggplot(NULL, aes(x,y)) +
  xlab("t [s]") + ylab("") + 
  labs(title = "Task Evolution (f=20, t=5, K=0)") +
  geom_line(data = data.frame(x=t, y=de), aes(colour="blue")) +
  geom_line(data = data.frame(x=t_red, y=dde), aes(colour="red")) +
  geom_line(data = data.frame(x=t_red, y=ddde), aes(colour="green"))

#ddde_dt <- 0.5 * (ddde[1:(length(ddde)-1)] + ddde[2:(length(ddde))]) * dt_red[1:(length(dt_red)-1)]

sigma <- sum(abs(ddde))
print(paste("Sigma/C = ", sigma/C))









