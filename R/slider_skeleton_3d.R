# plot_skeleton_3D()
#
# NSL Analysis (https://github.com/alinen/rit-nsl-analysis)
# Aline Normoyle, anormoyle@brynmawr.edu
#
# 3D Animation of positions data

slider_skeleton_3d = function(D, participant, startFrame, endFrame) {

# We'll be using plotly for this, so let's require it
if (!requireNamespace("plotly", quietly = TRUE)) {
  stop("Package \"plotly\" needed for this function to work. Please install it.", call. = FALSE)
} else {
  library("plotly")
}

frames <- startFrame:endFrame
ids <- c(rwrist, relbow, rshoulder, neck, head, neck, lshoulder, lelbow, lwrist)

x <- D[frames,(ids-1)*3+1]
y <- D[frames,(ids-1)*3+2]
z <- D[frames,(ids-1)*3+3]
f <- rep(frames, length(ids))

fig <- plot_ly(x = as.vector(unlist(x)), 
               y = as.vector(unlist(y)), 
               z = as.vector(unlist(z)), 
               frame = f, 
               type = 'scatter3d', mode = 'lines+markers', showlegend = F,  
               line = list(width = 6, color = 'rgb(44, 160, 44)'),
               marker = list(size = 3.5, color = 'rgb(0, 0, 255)'))

fig <- fig %>%
  layout(
    scene= list(
      camera= list(
        eye = list(x=0, y = 0.5, z = -2.5),
        up= list(x= 0, y= 1, z= 0)
      ),
      xaxis= list(
        range = c(min(x),max(x))
      ),
      yaxis= list(
        range = c(min(y), max(y))
      ),
      zaxis= list(
        range = c(min(z), max(z))
      )
    )
  )

text = NULL;
for (i in frames) {
  text <- c(text, sprintf("%s %d (%.2fs)", participant, i, i/30))
}
fig <- add_text(fig, x = -0.1, y = 5, text=text, frame=frames, inherit=F, 
                textfont = list(color = '#000000', size = 24)) %>%
                layout(
                  xaxis=list(showgrid=F, zeroline=F, visible=F), 
                  yaxis=list(showgrid=F, zeroline=F, visible=F))
fig <- hide_legend(fig)

fig
return(fig)
}