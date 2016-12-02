library(ggplot2)



### Calculate C
tf <- 5
e0 <- 0.0668341
C <- 40/sqrt(3) * e0/(tf*tf)



### Load raw data
rawdata <- read.csv("csv_data/f5t5K10.csv", header=FALSE)
Jdata <- rawdata[rawdata$V2=='J',]
qdotdata <- rawdata[rawdata$V2=='qdot',]
t <- Jdata[,c("V1")]



### Calculate dt
dt <- c()
dt__ <- t[1]
for (t_ in t) {
  dt <- c(dt, t_ - dt__)
  dt__ <- t_
}
dt[1] <- dt[2]

#val <- dt[910]/2
#dt[909] <- val
#dt[910] <- val


### Calculate de
de <- c()
for (time in t) {
  J_ <- Jdata[Jdata$V1==time, 3:20]
  qdot_ <- qdotdata[qdotdata$V1==time, 3:20]
  de_ <- sum(J_ * qdot_)
  de <- c(de, de_)
}



### Calculate dde
dde <- c()
de__ <- 0#de[1]
i <- 1
for (de_ in de) {
  val <- (de_ - de__)/dt[i]
  #if (i==909) {
  #  print(de_)
  #  print(de__)
  #  print(dt[i])
  #  print(val)
  #}
  dde <- c(dde, val)
  de__ <- de_
  i <- i + 1
}
#dde[1] <- dde[2]


### Calculate ddde
ddde <- c()
dde__ <- 0#dde[1]
i <- 1
for (dde_ in dde) {
  ddde <- c(ddde, (dde_ - dde__)/dt[i])
  dde__ <- dde_
  i <- i + 1
}
#ddde[length(ddde)] <- 0



## Calculate sigma
sigma <- sum(abs(ddde*dt))
rate <- sigma/C

ggplot(data.frame(x=t,y=de),aes(x,y)) + geom_line()
ggplot(data.frame(x=t,y=dde),aes(x,y)) + geom_line()
ggplot(data.frame(x=t,y=ddde),aes(x,y)) + geom_line()
