

/*
for (int t = 0; t < tmpDataPos.size(); t++) {
	//初始化矩阵
	if (t == 0) {
		//setEstimate(initObs);
		initFlag = 1;
	}

	//执行nnda的操作
	Update(tmpDataPos.at(t));
	cout << "tmpDataPos.at(t):" << tmpDataPos.at(t) << endl; //tmpData.at(t)为 2行num列 的矩阵
	cout << "tmpDataPos.at(t).rows():" << tmpDataPos.at(t).rows() << endl; //行
	cout << "tmpDataPos.at(t).cols():" << tmpDataPos.at(t).cols() << endl; //列
	int measureNum = tmpDataPos.at(t).cols();
	for (int i = 0; i < measureNum; i++) {
		for (int j = 0; j < measureNum; j++) {
			MatrixXd d = tmpDataPos.at(t).cols(j) - tmpDataPos.at(t).cols(i);
		}
	}


	initFlag = 0;

	//以下为执行写入数据到文件操作
	vector<MatrixXd> data;

}
*/