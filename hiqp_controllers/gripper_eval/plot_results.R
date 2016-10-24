library(ggplot2)

rawdata <- read.csv("results_old.csv", header=FALSE)

startpoint_map <- aggregate(rawdata$V7, list(rawdata$V2, rawdata$V3), function (x) {x=='success'})
n_testcases <- c()
success_counter <- c()
for (i in 1:nrow(startpoint_map)) {
  success_counter <- c( success_counter, sum( unlist(startpoint_map[i,]$x) ) )
  n_testcases <- c( n_testcases, length( unlist(startpoint_map[i,]$x) ) )
}
startpoint_map <- cbind(startpoint_map, success_counter, n_testcases, success_counter/n_testcases)

endpoint_map <- aggregate(rawdata$V7, list(rawdata$V4, rawdata$V5), function (x) {x=='success'})
n_testcases <- c()
success_counter <- c()
for (i in 1:nrow(endpoint_map)) {
  success_counter <- c( success_counter, sum( unlist(endpoint_map[i,]$x) ) )
  n_testcases <- c( n_testcases, length( unlist(endpoint_map[i,]$x) ) )
}
endpoint_map <- cbind(endpoint_map, success_counter, n_testcases, success_counter/n_testcases)









