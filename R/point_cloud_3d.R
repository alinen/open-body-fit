
point_cloud_3d = function(positions, participant, sf, ef) {

frame = floor(0.5*(sf+ef))
fig <- plot_skeleton_3d(positions, participant, frame)
fig <- fig %>% add_trace(x=as.vector(unlist(positions$lwristX[sf:ef])), 
                         y=as.vector(unlist(positions$lwristY[sf:ef])), 
                         z=as.vector(unlist(positions$lwristZ[sf:ef])), 
                         type="scatter3d", mode="lines+markers", inherit=F,
                         marker = list(size = 4, color = '#3200F040'),
                         line = list(size = 1, color = '#3200F040'),
                         name = "Left")
fig <- fig %>% add_trace(x=as.vector(unlist(positions$rwristX[sf:ef])), 
                         y=as.vector(unlist(positions$rwristY[sf:ef])), 
                         z=as.vector(unlist(positions$rwristZ[sf:ef])), 
                         type="scatter3d", mode="lines+markers", inherit=F,
                         marker = list(size = 4, color = '#80000040'),
                         line = list(size = 1, color = '#80000040'),
                         name = "Right")
fig

return(fig)
}