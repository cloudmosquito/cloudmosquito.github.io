# Attention


$$\text{Attention} = \text{softmax}\left(\frac{QK^{\top}}{\sqrt{d_k}}\right)V$$

我们以 3 个输入向量，维度 512 为例，$Q = \begin{pmatrix}q_1\\ q_2\\ q_3\end{pmatrix}, K = \begin{pmatrix}k_1\\ k_2\\ k_3\end{pmatrix}, V = \begin{pmatrix}v_1\\ v_2\\ v_3\end{pmatrix}$ . 这其中，Q 代表三个输入向量的查询，K 代表三个输入向量的键值，V 代表三个输入向量的信息。

$$QK^{\top} = \begin{pmatrix}q_1k_1^{\top} & q_1k_2^{\top} & q_1k_3^{\top}\\
q_2k_1^{\top} & q_2k_2^{\top} & q_2k_3^{\top}\\
q_3k_1^{\top} & q_3k_2^{\top} & q_3k_3^{\top}\\
\end{pmatrix}$$

除以 $\sqrt{d_k}$ 是为了便于学习。

Attention 的 softmax 是 **按行** 计算的，每一行的结果和为 1 ，这反映了每个键 key 与单个查询 query 的相关性。

举个例子，最终结果

$$\text{Attention} = \begin{pmatrix}0.1v_1+0.8v_2+0.1v_3\\ 0.2v_1+0.6v_2+0.2v_3\\0.9v_1+0.01v_2+0.09v_3\end{pmatrix}$$

其含义可以理解为，对于第一个输入向量，它应该重点关注（0.8）第二个输入向量，捎带关注第一（0.1）、第三个（0.1）输入向量，因此它需要的关键信息就是 $0.1v_1+0.8v_2+0.1v_3$ .