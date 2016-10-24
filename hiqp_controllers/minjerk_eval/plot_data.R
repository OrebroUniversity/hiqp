library(ggplot2)

source("multiplot.R")

rawdata <- read.csv("test_data.csv", header=FALSE)

series_t <- rawdata[rawdata$V2==5,]
series_f <- rawdata[rawdata$V1==20,]

f_tf <- series_f[,2]
f_efe0 <- series_f[,6]
f_SC <- series_f[,7]

t_f <- series_t[,1]
t_efe0 <- series_t[,6]
t_SC <- series_t[,7]

p1 <- ggplot(data.frame(x=t_f, y=t_efe0), aes(x=x, y=y)) + geom_line() +
  xlab("freq [Hz]") + ylab("final error [%]") + labs(title = "efe0 as function of f")

p2 <- ggplot(data.frame(x=t_f, y=t_SC), aes(x=x, y=y)) + geom_line() +
  xlab("freq [Hz]") + ylab("deviance from C") + labs(title = "SC as function of f")

p3 <- ggplot(data.frame(x=f_tf, y=f_efe0), aes(x=x, y=y)) + geom_line() +
  xlab("duration [s]") + ylab("final error [%]") + labs(title = "Rate of task fulfillment")

p4 <- ggplot(data.frame(x=f_tf, y=f_SC), aes(x=x, y=y)) + geom_line() +
  xlab("duration [s]") + ylab("Sigma over C") + labs(title = "Rate of jerk consistency")

multiplot(p3, p4, cols=2)

#grid.arrange(p1, p2, ncol = 2, main = "Main title")
