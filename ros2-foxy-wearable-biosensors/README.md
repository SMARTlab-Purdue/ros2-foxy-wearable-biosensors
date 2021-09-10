#  Wearable Biosensor Framework
Each package node follows a generalized structure as below figure. We categorized sensor data into three major types, using ROS 2 standard messages and separated per node. Hardware data indicates current battery levels and Bluetooth signal strength. Nodes publish raw data in real-time, with sampling rates based on the individual biosensor hardware specifications. 
Chunk data, collected per node with predefined lengths, provide end-users with a framework for downstream processing (e.g., feature engineering and \ac{ML} applications). 

# ROS2 Parameters
In each sensor node, there are three ROS 2 parameters (_Chuck_Enable_, _Chunk_Length_, _Sensor_Enable_).

1) _Sensor_Enable_:a boolean data type (i.e., _True_ or _False_).
2) _Chuck_Enable_: a boolean data type (i.e., _True_ or _False_).
3) _Chunk_Length_: a float data type to adjust data length per topic. 



Available topics depend on the individual biosensor hardware specifications. 


All package topic names follow the following format:\\

\begin{minipage}[t]{\textwidth}
  \centering
  \begin{minted}{python}
/biosensors/<sensor_name>/<data_name>
  \end{minted}
\end{minipage}
\newline

where the \textit{biosensor\_name} is the official name of the targeted biosensors (e.g., \textit{empatica\_e4})and \textit{data\_name} is the biosignal type (e.g., \textit{PPG\_raw} and \textit{PPG\_chunk}).
