library(ggplot2)

### Calculate C
tf <- 5
e0 <- 0.0668388
C <- 40/sqrt(3) * e0/(tf*tf)



### Load raw data
rawdata <- read.csv("csv_data/f5t5K10.csv", header=FALSE)
Jdata <- rawdata[rawdata$V2=='J',]
qdotdata <- rawdata[rawdata$V2=='qdot',]
t <- Jdata[,c("V1")]



### Calculate de
de <- c()
for (time in t) {
  J_ <- Jdata[Jdata$V1==time, 3:20]
  qdot_ <- qdotdata[qdotdata$V1==time, 3:20]
  de_ <- sum(J_ * qdot_)
  de <- c(de, de_)
}
#de_len <- length(de)
#de[de_len-1] <- (de[de_len-2]+de[de_len-3])/2
#de[de_len] <- (de[de_len-1]+de[de_len-2])/2


### Interpolate de to get an evenly distributed time series
moddata <- data.frame(t=t, de=de)
dt <- sum(t[2:length(t)]-t[1:length(t)-1])/(length(t)-1)
t_0 <- t[1]
t_f <- t[length(t)]
n <- ceiling((t_f-t_0)/dt)+1

t_ <- 0
t_ip <- c()
de_ip <- c()
for (i in 1:n) {
  if ((t_+t_0) %in% moddata$t) {
    val <- moddata$de[moddata$t==t_+t_0]
  } else { # we need to interpolate
    row1 <- moddata[moddata$t < t_+t_0,]
    row1 <- row1[nrow(row1),]
    row2 <- moddata[moddata$t > t_+t_0,][1,]
    t_len <- row2$t - row1$t
    f1 <- 1 - (t_+t_0 - row1$t)/t_len
    f2 <- 1-f1
    val <- row1$de * f1 + row2$de * f2
  }
  de_ip <- c(de_ip, val)
  t_ip <- c(t_ip, t_)
  t_ <- t_ + dt
}
intdata <- data.frame(t=t_ip, de=de_ip)
#ggplot(moddata, aes(x=t,y=de)) + geom_line()
#ggplot(intdata, aes(x=t,y=de)) + geom_line()



### W A R N I N G

### Cut the last entry as it screws up the numerical differentiation
cut_size <- 1
n <- n-cut_size
de_ip <- de_ip[1:(length(de_ip)-cut_size)]
t <- t[1:(length(t)-cut_size)]
t_ip <- t_ip[1:(length(t_ip)-cut_size)]





### Calculate first differentiation, dde
accuracy <- 8
d_forw_coeff8 <- c(-761/280, 8, -14, 56/3, -35/2, 56/5, -14/3, 8/7, -1/8)
d_back_coeff8 <- c(1/8, -8/7, 14/3, -56/5, 35/2, -56/3, 14, -8, 761/280)
d_cen_coeff2 <- c(-1/2, 0, 1/2)
#d_cen_coeff4 <- c(1/12, -2/3, 0, 2/3, -1/12)
#d_cen_coeff6 <- c(-1/60, 3/20, -3/4, 0, 3/4, -3/20, 1/60)
d_cen_coeff8 <- c(1/280, -4/105, 1/5, -4/5, 0, 4/5, -1/5, 4/105, -1/280)

dde <- c()
for (i in 1:(accuracy/2)) {
  #val <- sum( de_ip[i:(i+accuracy)]*d_forw_coeff8 ) / dt
  val <- 0
  dde <- c(dde, val)
}
for (i in (accuracy/2+1):(n-accuracy/2)) {
  val <- sum( de_ip[(i-accuracy/2):(i+accuracy/2)]*d_cen_coeff8 ) / dt
  dde <- c(dde, val)
}
for (i in (n-accuracy/2+1):n) {
  #val <- sum( de_ip[(i-accuracy):i]*d_back_coeff8 ) / dt
  val <- 0
  dde <- c(dde, val)
}

acceldata <- data.frame(t = t_ip, dde=dde)
ggplot(acceldata, aes(x=t,y=dde)) + geom_line()



#de_ip <- (8*t*t*t)

### Calculate second differentiation, ddde
#accuracy <- 8
#dd_forw_coeff7 <- c(29531/5040, -962/35, 621/10, -4006/45, 691/8, -282/5, 2143/90, -206/35, 363/560)
#dd_back_coeff7 <- -rev(dd_forw_coeff7)
#dd_cen_coeff8 <- c(-1/560, 8/315, -1/5, 8/5, -205/72, 8/5, -1/5, 8/315, -1/560)

#ddde <- c()
#for (i in 1:(accuracy/2)) {
#  val <- sum( de_ip[i:(i+accuracy)]*dd_forw_coeff7 ) / (dt*dt)
#  ddde <- c(ddde, val)
#}
#for (i in (accuracy/2+1):(n-accuracy/2)) {
#  val <- sum( de_ip[(i-accuracy/2):(i+accuracy/2)]*dd_cen_coeff8 ) / (dt*dt)
#  ddde <- c(ddde, val)
#}
#for (i in (n-accuracy/2+1):n) {
#  val <- sum( de_ip[(i-accuracy):i]*dd_back_coeff7 ) / (dt*dt)
#  ddde <- c(ddde, val)
#}

#jerkdata <- data.frame(t = t_ip, ddde=ddde)
#ggplot(jerkdata, aes(x=t,y=ddde)) + geom_line()




### Calculate Sigma

#t_q4 = (t_f-t_0)
#t_q1 <- t_q4*0.5*(1-1/sqrt(3))
#t_q3 <- t_q4*0.5*(1+1/sqrt(3))

#t_q1 <- t_ip[t_ip > t_q1][1]
#t_q3 <- t_ip[t_ip > t_q3][1]
#t_q4 <- t_ip[length(t_ip)]

#steps <- 0.606970852 * c(-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5)
#prob_map <- exp(-0.5*steps*steps)/sqrt(2*pi)
#prob_map <- prob_map / sum(prob_map)

#acc_ <- acceldata$dde[ acceldata$t>(t_q1-5*dt) & acceldata$t<(t_q1+5*dt) ]
#acc_q1 <- sum( acc_ * prob_map )



sigma <- sum( abs( acceldata$dde[2:nrow(acceldata)] - acceldata$dde[1:nrow(acceldata)-1] ) )
sigma <- sigma + cut_size * abs(acceldata$dde[nrow(acceldata)] - acceldata$dde[nrow(acceldata)-1])
rate <- sigma/C









